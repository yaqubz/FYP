#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include <stdbool.h>
#include <signal.h>
#include "nlink_linktrack_anchorframe0.h"
#include "nlink_utils.h"

#define BUFFER_SIZE 4096
#define DATA_LENGTH 896
#define START_MARKER 0x55
#define END_MARKER 0xEE
#define BYTES_PER_LINE 22

#define MAX_LINES 1000 // Keep only last 1000 entries in the .txt file

volatile bool stop_signal = false;

// Graceful exit handler
void handle_signal(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        stop_signal = true;
    }
}

HANDLE open_serial_port() {
    char port_name[20];
    HANDLE hComm;

    while (true) {
        printf("Enter COM port number (e.g., 11 for COM11): ");
        int com_num;
        scanf("%d", &com_num);
        snprintf(port_name, sizeof(port_name), "\\\\.\\COM%d", com_num);

        hComm = CreateFile(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

        if (hComm != INVALID_HANDLE_VALUE) {
            printf("Successfully connected to %s\n", port_name);
            return hComm;
        }

        printf("Error opening %s. Please try again.\n", port_name);
    }
}

void parseAnchorFrame0Data(const uint8_t *data, size_t data_length) {
    if (nlt_anchorframe0_.UnpackData(data, data_length)) {
        printf("Anchor Frame0 data unpacked successfully:\n");
        printf("Role: %d, ID: %d\n", nlt_anchorframe0_.result.role, nlt_anchorframe0_.result.id);
        printf("Local Time: %d, System Time: %d\n", nlt_anchorframe0_.result.local_time, nlt_anchorframe0_.result.system_time);
        printf("Voltage: %.2fV\n", nlt_anchorframe0_.result.voltage);

        printf("Valid Nodes: %d\n", nlt_anchorframe0_.result.valid_node_count);
        for (int i = 0; i < nlt_anchorframe0_.result.valid_node_count; ++i) {
            nlt_anchorframe0_node_t *node = nlt_anchorframe0_.result.nodes[i];
            printf("Node %d:\n", i);
            printf("  ID: %d, Role: %d\n", node->id, node->role);
            printf("  Position: [%.2f, %.2f, %.2f]\n", node->pos_3d[0], node->pos_3d[1], node->pos_3d[2]);
            printf("  Distances: ");
            for (int j = 0; j < 8; ++j) {
                printf("%.2f ", node->dis_arr[j]);
            }
            printf("\n");
        }
    } else {
        printf("Parse error\n");
    }
    printf("-------------------------------\n");
}

void trim_file(const char* filename) {
    FILE *fp = fopen(filename, "r");
    if (!fp) return;

    int line_count = 0;
    char ch;
    while (!feof(fp)) {
        ch = fgetc(fp);
        if (ch == '\n') {
            line_count++;
        }
    }
    fclose(fp);

    if (line_count > MAX_LINES) {
        FILE *fp = fopen(filename, "r");
        FILE *temp = fopen("temp.txt", "w");
        if (!fp || !temp) return;

        int lines_to_skip = line_count - MAX_LINES;
        for (int i = 0; i < lines_to_skip; i++) {
            while ((ch = fgetc(fp)) != '\n' && ch != EOF);
        }

        while ((ch = fgetc(fp)) != EOF) {
            fputc(ch, temp);
        }

        fclose(fp);
        fclose(temp);

        remove(filename);
        rename("temp.txt", filename);
    }
}

void write_data_to_file(nlt_anchorframe0_node_t *node) {
    const char* filename = "uwb_data.txt";
    FILE *fp = fopen(filename, "a");
    if (!fp) return;

    fprintf(fp, "%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            node->id, node->role,
            node->pos_3d[0], node->pos_3d[1], node->pos_3d[2],
            node->dis_arr[0], node->dis_arr[1], node->dis_arr[2], node->dis_arr[3],
            node->dis_arr[4], node->dis_arr[5], node->dis_arr[6], node->dis_arr[7]);

    fclose(fp);

    static int write_count = 0;
    write_count++;
    if (write_count >= 100) {
        trim_file(filename);
        write_count = 0;
    }
}

void process_frame(const uint8_t *data, size_t length) {
    printf("\nProcessing frame (%zu bytes):\n", length);
    parseAnchorFrame0Data(data, length);
    if (nlt_anchorframe0_.UnpackData(data, length)) {
        for (int i = 0; i < nlt_anchorframe0_.result.valid_node_count; ++i) {
            write_data_to_file(nlt_anchorframe0_.result.nodes[i]);
        }
    }
}

int main() {
    HANDLE hComm;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
    uint8_t buffer[BUFFER_SIZE] = {0};
    size_t buffer_pos = 0;
    DWORD bytes_read;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    hComm = open_serial_port();

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    GetCommState(hComm, &dcbSerialParams);
    dcbSerialParams.BaudRate = 921600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.Parity = NOPARITY;
    dcbSerialParams.StopBits = ONESTOPBIT;
    SetCommState(hComm, &dcbSerialParams);

    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.ReadTotalTimeoutConstant = 1;
    SetCommTimeouts(hComm, &timeouts);

    PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
    printf("Reading frames...\n");

    while (!stop_signal) {
        BOOL read_result = ReadFile(hComm, buffer + buffer_pos, 1, &bytes_read, NULL);

        if (!read_result) {
            DWORD error = GetLastError();
            printf("ReadFile error: %lu\n", error);
            continue;
        }

        if (bytes_read > 0) {
            buffer_pos++;

            if (buffer_pos >= DATA_LENGTH) {
                for (size_t i = 0; i <= buffer_pos - DATA_LENGTH; i++) {
                    if (buffer[i] == START_MARKER && buffer[i + DATA_LENGTH - 1] == END_MARKER) {
                        process_frame(buffer + i, DATA_LENGTH);
                        memmove(buffer, buffer + i + DATA_LENGTH, buffer_pos - (i + DATA_LENGTH));
                        buffer_pos -= (i + DATA_LENGTH);
                        break;
                    }
                }
            }

            if (buffer_pos >= BUFFER_SIZE - 1) {
                buffer_pos = 0;
            }
        } else {
            Sleep(1);
        }
    }

    printf("Closing serial port...\n");
    CloseHandle(hComm);
    printf("Program terminated gracefully.\n");

    return 0;
}
