// 8 Mar Updated - USE THIS! Can tune params to get best data. Original publishing rate = 66Hz

#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include <stdbool.h>
#include <signal.h>
#include <winsock.h>
#include <math.h>
#include "nlink_linktrack_anchorframe0.h"
#include "nlink_utils.h"

#pragma comment(lib, "ws2_32.lib") // Link Winsock library

#define BUFFER_SIZE 4096
#define DATA_LENGTH 896
#define START_MARKER 0x55
#define END_MARKER 0xEE
#define BYTES_PER_LINE 22

// TUNE THESE PARAMS IF NEEDED!
#define PUBLISHING_RATE_HZ 20  // Max seems to be 12-15Hz
#define MEDIAN_WINDOW 7      // Size of median window
#define MOVING_AVG_WINDOW 10   // Size of moving average window
#define MAX_JUMP 0.5          // Maximum allowed position jump in meters
#define DECAY_TIME_MS 2000    // Time after which the filter resets if no valid data is received

volatile bool stop_signal = false;
SOCKET clientSocket;
struct sockaddr_in serverAddr;

// Structure to store position history and last valid position
typedef struct {
    float window[MEDIAN_WINDOW];
    int window_idx;
    float last_valid_pos;
    DWORD last_valid_time; // Timestamp of the last valid position
    bool initialized;
} PositionFilter;

// Structure for moving average filter
typedef struct {
    float window[MOVING_AVG_WINDOW];
    int window_idx;
    float sum;
    bool initialized;
} MovingAvgFilter;

// Global filter objects for each dimension
PositionFilter x_median_filter, y_median_filter, z_median_filter;
MovingAvgFilter x_avg_filter, y_avg_filter, z_avg_filter;

// Buffer to store the most recent filtered data
typedef struct {
    int id;
    int role;
    float pos_3d[3];
    float dis_arr[8];
} FilteredNodeData;

FilteredNodeData latest_filtered_data;
bool new_filtered_data_available = false;

// Graceful exit handler
void handle_signal(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        stop_signal = true;
    }
}

// Function to sort an array (simple bubble sort for small arrays)
void bubble_sort(float arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

// Initialize the median filter
void init_median_filter(PositionFilter *filter) {
    for (int i = 0; i < MEDIAN_WINDOW; i++) {
        filter->window[i] = 0.0f;
    }
    filter->window_idx = 0;
    filter->last_valid_pos = 0.0f;
    filter->last_valid_time = GetTickCount();
    filter->initialized = false;
}

// Initialize the moving average filter
void init_moving_avg_filter(MovingAvgFilter *filter) {
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
        filter->window[i] = 0.0f;
    }
    filter->window_idx = 0;
    filter->sum = 0.0f;
    filter->initialized = false;
}

// Apply median filter with outlier rejection and decay mechanism
float apply_median_filter(PositionFilter *filter, float new_pos) {
    DWORD current_time = GetTickCount();

    // Check if the new position is NaN or infinity
    if (isnan(new_pos) || isinf(new_pos)) {
        return filter->last_valid_pos;
    }

    // If not initialized, accept the first position
    if (!filter->initialized) {
        for (int i = 0; i < MEDIAN_WINDOW; i++) {
            filter->window[i] = new_pos;
        }
        filter->last_valid_pos = new_pos;
        filter->last_valid_time = current_time;
        filter->initialized = true;
        return new_pos;
    }

    // Check if new position is a physically impossible jump
    if (fabsf(new_pos - filter->last_valid_pos) > MAX_JUMP) {
        // If the filter is stuck for too long, reset it
        if (current_time - filter->last_valid_time > DECAY_TIME_MS) {
            init_median_filter(filter);
            return new_pos;
        }
        // Otherwise, return the last valid position
        return filter->last_valid_pos;
    }

    // Update window with new position
    filter->window[filter->window_idx] = new_pos;
    filter->window_idx = (filter->window_idx + 1) % MEDIAN_WINDOW;

    // Calculate median
    float sorted[MEDIAN_WINDOW];
    memcpy(sorted, filter->window, sizeof(float) * MEDIAN_WINDOW);
    bubble_sort(sorted, MEDIAN_WINDOW);

    // Update last valid position and timestamp
    filter->last_valid_pos = sorted[MEDIAN_WINDOW / 2];
    filter->last_valid_time = current_time;

    return filter->last_valid_pos;
}

// Apply moving average filter
float apply_moving_avg_filter(MovingAvgFilter *filter, float new_pos) {
    if (!filter->initialized) {
        for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
            filter->window[i] = new_pos;
        }
        filter->sum = new_pos * MOVING_AVG_WINDOW;
        filter->initialized = true;
        return new_pos;
    }

    // Subtract the oldest value and add the new value
    filter->sum -= filter->window[filter->window_idx];
    filter->sum += new_pos;
    filter->window[filter->window_idx] = new_pos;
    filter->window_idx = (filter->window_idx + 1) % MOVING_AVG_WINDOW;

    return filter->sum / MOVING_AVG_WINDOW;
}

void process_frame(const uint8_t *data, size_t data_length) {
    if (nlt_anchorframe0_.UnpackData(data, data_length)) {
        for (int i = 0; i < nlt_anchorframe0_.result.valid_node_count; ++i) {
            nlt_anchorframe0_node_t *node = nlt_anchorframe0_.result.nodes[i];

            // Apply median filter
            float median_x = apply_median_filter(&x_median_filter, node->pos_3d[0]);
            float median_y = apply_median_filter(&y_median_filter, node->pos_3d[1]);
            float median_z = apply_median_filter(&z_median_filter, node->pos_3d[2]);

            // Apply moving average filter
            float filtered_x = apply_moving_avg_filter(&x_avg_filter, median_x);
            float filtered_y = apply_moving_avg_filter(&y_avg_filter, median_y);
            float filtered_z = apply_moving_avg_filter(&z_avg_filter, median_z);

            // Store the filtered data
            latest_filtered_data.id = node->id;
            latest_filtered_data.role = node->role;
            latest_filtered_data.pos_3d[0] = filtered_x;
            latest_filtered_data.pos_3d[1] = filtered_y;
            latest_filtered_data.pos_3d[2] = filtered_z;
            memcpy(latest_filtered_data.dis_arr, node->dis_arr, sizeof(float) * 8);

            // Mark new filtered data as available
            new_filtered_data_available = true;
        }
    } else {
        printf("Parse error\n");
    }
}

void publish_filtered_data() {
    if (new_filtered_data_available) {
        char buffer[2048] = {0};
        int total_len = 0;

        int len = snprintf(buffer + total_len, sizeof(buffer) - total_len,
                "%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                latest_filtered_data.id,
                latest_filtered_data.role,
                latest_filtered_data.pos_3d[0],
                latest_filtered_data.pos_3d[1],
                latest_filtered_data.pos_3d[2],
                latest_filtered_data.dis_arr[0],
                latest_filtered_data.dis_arr[1],
                latest_filtered_data.dis_arr[2],
                latest_filtered_data.dis_arr[3],
                latest_filtered_data.dis_arr[4],
                latest_filtered_data.dis_arr[5],
                latest_filtered_data.dis_arr[6],
                latest_filtered_data.dis_arr[7]
                );

        if (len > 0) {
            total_len += len;
        }

        if (total_len > 0) {
            int result = sendto(clientSocket, buffer, total_len, 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
            if (result == SOCKET_ERROR) {
                printf("Failed to send broadcast with error: %d\n", WSAGetLastError());
            } else {
                printf("Successfully sent %d bytes\n", result);
            }
        }

        new_filtered_data_available = false;
    }
}

bool init_socket_broadcast(int port) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        printf("WSAStartup failed with error: %d\n", WSAGetLastError());
        return false;
    }

    clientSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (clientSocket == INVALID_SOCKET) {
        printf("Socket creation failed with error: %d\n", WSAGetLastError());
        WSACleanup();
        return false;
    }

    // Enable broadcasting
    BOOL broadcast = TRUE;
    if (setsockopt(clientSocket, SOL_SOCKET, SO_BROADCAST, (char*)&broadcast, sizeof(broadcast)) < 0) {
        printf("Failed to set socket as broadcast with error: %d\n", WSAGetLastError());
        closesocket(clientSocket);
        WSACleanup();
        return false;
    }

    // Clear the structure
    memset(&serverAddr, 0, sizeof(serverAddr));
    
    // Set up the broadcast address
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);  // Proper network byte order

    printf("Socket initialized for broadcasting on port %d\n", port);
    return true;
}

void cleanup_socket() {
    closesocket(clientSocket);
    WSACleanup();
}

HANDLE open_com_port() {
    char port_name[20];
    int com_num;
    
    printf("Enter COM port number (e.g., 11 for COM11): ");
    if (scanf("%d", &com_num) != 1) {
        printf("Invalid input. Program will exit.\n");
        return INVALID_HANDLE_VALUE;
    }
    
    snprintf(port_name, sizeof(port_name), "\\\\.\\COM%d", com_num);
    printf("Attempting to connect to %s...\n", port_name);
    
    HANDLE hComm = CreateFile(port_name, 
                            GENERIC_READ | GENERIC_WRITE, 
                            0, 
                            NULL, 
                            OPEN_EXISTING, 
                            FILE_ATTRIBUTE_NORMAL, 
                            NULL);
    
    if (hComm == INVALID_HANDLE_VALUE) {
        printf("Error: Unable to open %s\n", port_name);
    } else {
        printf("Successfully connected to %s\n", port_name);
    }
    
    return hComm;
}

int main() {
    HANDLE hComm;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
    uint8_t buffer[BUFFER_SIZE] = {0};
    size_t buffer_pos = 0;
    DWORD bytes_read;

    // Set up signal handling
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    
    // Initialize the median filters
    init_median_filter(&x_median_filter);
    init_median_filter(&y_median_filter);
    init_median_filter(&z_median_filter);

    // Initialize the moving average filters
    init_moving_avg_filter(&x_avg_filter);
    init_moving_avg_filter(&y_avg_filter);
    init_moving_avg_filter(&z_avg_filter);

    // Initialize socket
    int port = 5000;
    if (!init_socket_broadcast(port)) {
        printf("Failed to initialize socket\n");
        return 1;
    }

    // Get COM port from user and try to open it
    hComm = open_com_port();
    if (hComm == INVALID_HANDLE_VALUE) {
        cleanup_socket();
        return 1;
    }

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

    // Variable to control the 2 Hz frequency
    DWORD last_send_time = GetTickCount();

    while (!stop_signal) {
        BOOL read_result = ReadFile(hComm, buffer + buffer_pos, 1, &bytes_read, NULL);

        if (!read_result) {
            DWORD error = GetLastError();
            printf("ReadFile error: %lu\n", error);
            continue;
        }

        if (bytes_read > 0) {
            buffer_pos++;

            // Check for a complete frame
            if (buffer_pos >= DATA_LENGTH) {
                for (size_t i = 0; i <= buffer_pos - DATA_LENGTH; i++) {
                    if (buffer[i] == START_MARKER && buffer[i + DATA_LENGTH - 1] == END_MARKER) {
                        // Process the frame (apply filters)
                        process_frame(buffer + i, DATA_LENGTH);

                        // Remove processed data from the buffer
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
            Sleep(1); // Avoid busy looping
        }

        // Check if 500 ms has passed since the last send (for 2Hz rate)
        DWORD current_time = GetTickCount();
        if (current_time - last_send_time >= 1000/PUBLISHING_RATE_HZ) { 

            float delta_time = (current_time - last_send_time) / 1000.0f;  // Convert milliseconds to seconds
            float publishing_frequency = 1.0f / delta_time;  // Calculate frequency in Hz

            // Print the publishing frequency
            printf("Publishing data at %.2f Hz...\n", publishing_frequency);
            
            // Publish the latest filtered data
            publish_filtered_data();
            last_send_time = current_time;
        }
    }

    printf("Closing serial port...\n");
    CloseHandle(hComm);

    cleanup_socket();
    printf("Program terminated gracefully.\n");

    return 0;
}