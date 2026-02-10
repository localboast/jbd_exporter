#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <mosquitto.h>
#include <cjson/cJSON.h>

// --- Configuration Struct ---
typedef struct {
    char serial_port[64];
    char mqtt_broker[64];
    int mqtt_port;
    char mqtt_user[64];
    char mqtt_password[64];
    int poll_interval;
    char device_name[64];
    char device_id[64];
} AppConfig;

AppConfig config;

// --- Serial Port Setup ---
int setup_serial(const char *portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    cfmakeraw(&tty); // Set raw mode for clean 8-bit data
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 20; // 2.0 seconds read timeout
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

// --- JBD Protocol Helpers ---
uint16_t jbd_checksum(uint8_t *data, int len) {
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint16_t)((0x10000 - sum) & 0xFFFF);
}

int send_and_read(int fd, uint8_t reg, uint8_t *out_buffer, int max_len) {
    tcflush(fd, TCIFLUSH);
    uint8_t cmd[7] = {0xDD, 0xA5, reg, 0x00, 0x00, 0x00, 0x77};
    uint16_t cs = jbd_checksum(&cmd[2], 2);
    cmd[4] = (cs >> 8) & 0xFF; cmd[5] = cs & 0xFF;
    if (write(fd, cmd, 7) != 7) return -1;
    usleep(150000); // Wait for BMS to process
    int total_read = 0, aligned = 0;
    uint8_t header[4];
    while (total_read < 4) {
        uint8_t byte;
        if (read(fd, &byte, 1) <= 0) return -2; // Timeout
        if (!aligned) { if (byte == 0xDD) { aligned = 1; header[0] = byte; total_read = 1; } }
        else header[total_read++] = byte;
    }
    if (header[2] != 0x00) return -3; // BMS Error
    int data_len = header[3], expected = data_len + 3;
    if (4 + expected > max_len) return -4;
    memcpy(out_buffer, header, 4);
    int rem_read = 0;
    while(rem_read < expected) {
        int n = read(fd, out_buffer + 4 + rem_read, expected - rem_read);
        if (n <= 0) return -2;
        rem_read += n;
    }
    return 4 + expected;
}

// --- Config File Parser ---
void load_config(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) return;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char *key = strtok(line, "=");
        char *val = strtok(NULL, "\n");
        if (!key || !val) continue;
        if (strcmp(key, "serial_port") == 0) strcpy(config.serial_port, val);
        else if (strcmp(key, "mqtt_broker") == 0) strcpy(config.mqtt_broker, val);
        else if (strcmp(key, "mqtt_port") == 0) config.mqtt_port = atoi(val);
        else if (strcmp(key, "mqtt_user") == 0) strcpy(config.mqtt_user, val);
        else if (strcmp(key, "mqtt_password") == 0) strcpy(config.mqtt_password, val);
        else if (strcmp(key, "poll_interval") == 0) config.poll_interval = atoi(val);
        else if (strcmp(key, "device_name") == 0) strcpy(config.device_name, val);
        else if (strcmp(key, "device_id") == 0) strcpy(config.device_id, val);
    }
    fclose(f);
}

// --- MQTT Discovery Publisher ---
void publish_config(struct mosquitto *mosq, const char *sid, const char *name, const char *unit, const char *cls) {
    cJSON *root = cJSON_CreateObject();
    char topic[256], val_tmpl[128], uniq[128], stat[128];
    snprintf(topic, 256, "homeassistant/sensor/%s/%s/config", config.device_id, sid);
    snprintf(val_tmpl, 128, "{{ value_json.%s }}", sid);
    snprintf(uniq, 128, "%s_%s", config.device_id, sid);
    snprintf(stat, 128, "%s/status", config.device_id);
    cJSON_AddStringToObject(root, "name", name);
    cJSON_AddStringToObject(root, "state_topic", stat);
    cJSON_AddStringToObject(root, "value_template", val_tmpl);
    cJSON_AddStringToObject(root, "unique_id", uniq);
    if(unit) cJSON_AddStringToObject(root, "unit_of_measurement", unit);
    if(cls) cJSON_AddStringToObject(root, "device_class", cls);
    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", config.device_id);
    cJSON_AddStringToObject(dev, "name", config.device_name);
    cJSON_AddStringToObject(dev, "manufacturer", "JBD");
    cJSON_AddItemToObject(root, "device", dev);
    char *p = cJSON_PrintUnformatted(root);
    mosquitto_publish(mosq, NULL, topic, strlen(p), p, 0, true);
    free(p); cJSON_Delete(root);
}

// --- Main Application ---
int main() {
    // Set default values
    strcpy(config.serial_port, "/dev/ttyUSB1");
    strcpy(config.mqtt_broker, "127.0.0.1");
    config.mqtt_port = 1883;
    config.poll_interval = 10;
    strcpy(config.device_name, "JBD BMS");
    strcpy(config.device_id, "jbd_bms"); // Default matches your preference
    config.mqtt_user[0] = '\0';
    config.mqtt_password[0] = '\0';

    load_config("config.ini");

    printf("--- JBD Exporter Starting ---\n");
    printf("Device ID: '%s' | Serial Port: '%s'\n", config.device_id, config.serial_port);

    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new(config.device_id, true, NULL);
    if (strlen(config.mqtt_user) > 0) mosquitto_username_pw_set(mosq, config.mqtt_user, config.mqtt_password);
    if (mosquitto_connect(mosq, config.mqtt_broker, config.mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
        printf("MQTT Connection Failed.\n"); return 1;
    }
    mosquitto_loop_start(mosq);

    int fd = setup_serial(config.serial_port);
    if (fd < 0) return 1;

    uint8_t buf[512];
    int discovery_done = 0;
    char status_topic[128];
    snprintf(status_topic, 128, "%s/status", config.device_id); // Build the status topic ONCE

    while(1) {
        if (send_and_read(fd, 0x03, buf, sizeof(buf)) > 0) {
            cJSON *json = cJSON_CreateObject();
            uint8_t *d = &buf[4];
            int cells = d[21], ntcs = d[22];
            cJSON_AddNumberToObject(json, "pack_voltage", ((d[0]<<8)|d[1])/100.0);
            cJSON_AddNumberToObject(json, "pack_current", ((int16_t)((d[2]<<8)|d[3]))/100.0);
            cJSON_AddNumberToObject(json, "soc", d[19]);

            for(int i=0; i<ntcs; i++) {
                char kStr[16]; snprintf(kStr, 16, "temp_%d", i+1);
                uint16_t k = (d[23+i*2]<<8)|d[23+i*2+1];
                cJSON_AddNumberToObject(json, kStr, (k*0.1)-273.15);
            }

            if (send_and_read(fd, 0x04, buf, sizeof(buf)) > 0) {
                for(int i=0; i<cells; i++) {
                    char cStr[16]; snprintf(cStr, 16, "cell_%d", i+1);
                    uint16_t mv = (buf[4+i*2]<<8)|buf[4+i*2+1];
                    cJSON_AddNumberToObject(json, cStr, mv/1000.0);
                }
            }
            char *pl = cJSON_PrintUnformatted(json);
            mosquitto_publish(mosq, NULL, status_topic, strlen(pl), pl, 0, false);
            printf("Status sent to '%s': SOC %d%%\n", status_topic, d[19]);
            free(pl); cJSON_Delete(json);

            if (!discovery_done && cells > 0) {
                publish_config(mosq, "pack_voltage", "Pack Voltage", "V", "voltage");
                publish_config(mosq, "pack_current", "Pack Current", "A", "current");
                publish_config(mosq, "soc", "SOC", "%", "battery");
                for(int i=0; i<cells; i++) { char s[16], n[32]; snprintf(s, 16, "cell_%d", i+1); snprintf(n, 32, "Cell %d Voltage", i+1); publish_config(mosq, s, n, "V", "voltage"); }
                for(int i=0; i<ntcs; i++) { char s[16], n[32]; snprintf(s, 16, "temp_%d", i+1); snprintf(n, 32, "Temp %d", i+1); publish_config(mosq, s, n, "°C", "temperature"); }
                discovery_done = 1;
            }
        } else {
            printf("Read from BMS failed. Retrying...\n");
        }
        sleep(config.poll_interval);
    }
    return 0;
}
