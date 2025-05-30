#include "stm32f4xx.h"         // Device header
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mbedtls-bef26f687287/mbedtls/sha256.h"
#include "sha256.c"
#include "allthefunctions.h"

#define MAX_BUFFER 256
#define MAX_NONCE 100000000000ULL

static char new_hash[65];  // Final hash (64 chars + null terminator)

// --- SHA256 Hashing Function ---
static void SHA256_hash(const char* text, unsigned char* hash_bin) {
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    mbedtls_sha256_update(&ctx, (const unsigned char*)text, strlen(text));
    mbedtls_sha256_finish(&ctx, hash_bin);
    mbedtls_sha256_free(&ctx);
}

// --- Convert Binary Hash to Hex String ---
static void bin2hex(const unsigned char *bin, char *hex, size_t len) {
    for (size_t i = 0; i < len; i++) {
        sprintf(hex + (i * 2), "%02x", bin[i]);
    }
    hex[len * 2] = '\0';
}

// --- Check if Hash Starts with N Zeros ---
static int starts_with(const char* str, int prefix_zeros) {
    for (int i = 0; i < prefix_zeros; i++) {
        if (str[i] != '0') return 0;
    }
    return 1;
}

// --- Mining Function ---
static char* mine(int block_number, const char* transactions, const char* previous_hash, int prefix_zeros, char* hash_result) {
    char text[1024];
    unsigned char hash_bin[32];
    char hash_str[65];
    unsigned long long nonce;

    for (nonce = 0; nonce < MAX_NONCE; nonce++) {
        snprintf(text, sizeof(text), "%d%s%s%llu", block_number, transactions, previous_hash, nonce);
        SHA256_hash(text, hash_bin);
        bin2hex(hash_bin, hash_str, 32);

        if (starts_with(hash_str, prefix_zeros)) {
            char buf[64];
            sprintf(buf, "Nonce: %llu\r\n", nonce);
            UART1_SendString(buf);
						strcpy(hash_result, hash_str);
            return hash_result;
        }

        // Optional: feedback every 100000 tries
    }
    return NULL;
}

// --- Pad the previous hash to 64 characters ---
void pad_previous_hash(char* hash) {
    int len = strlen(hash);
    if (len < 64) {
        memset(hash + len, '0', 64 - len);  // pad with '0'
        hash[64] = '\0';
    }
}

// --- Main Function ---
int main(void) {
    SystemInit();
    UART1_Init();

    char transactions[MAX_BUFFER];
    char previous_hash[65];
    char input_buffer[32];
    int block_number;
    int difficulty;

    UART1_SendString("Enter transactions (e.g., George->Brown->100,...):\r\n");
    UART1_ReceiveLine(transactions, MAX_BUFFER);

    UART1_SendString("Enter block number:\r\n");
    UART1_ReceiveLine(input_buffer, sizeof(input_buffer));
    block_number = atoi(input_buffer);

    UART1_SendString("Enter difficulty level:\r\n");
    UART1_ReceiveLine(input_buffer, sizeof(input_buffer));
    difficulty = atoi(input_buffer);

    UART1_SendString("Enter previous hash (max 64 chars):\r\n");
    UART1_ReceiveLine(previous_hash, 65);
    pad_previous_hash(previous_hash);

    // Echo input back
    UART1_SendString("\r\n--- Input Summary ---\r\n");
    UART1_SendString("Transactions: "); UART1_SendString(transactions); UART1_SendString("\r\n");
    sprintf(input_buffer, "%d", block_number);
    UART1_SendString("Block Number: "); UART1_SendString(input_buffer); UART1_SendString("\r\n");
    sprintf(input_buffer, "%d", difficulty);
    UART1_SendString("Difficulty: "); UART1_SendString(input_buffer); UART1_SendString("\r\n");
    UART1_SendString("Previous Hash: "); UART1_SendString(previous_hash); UART1_SendString("\r\n");
    UART1_SendString("----------------------\r\n");

    UART1_SendString("Start mining...\r\n");
		Timer5_Init();
		Timer5_Reset();
		int time;
		char *result = mine(block_number, transactions, previous_hash, difficulty, new_hash);
		time= Timer5_Elapsed();
		sprintf(input_buffer, "TimeTaken %d ms\r\n", time);
		UART1_SendString(input_buffer);
		if (result != NULL) {
        UART1_SendString("Found hash: ");
        UART1_SendString(result);
        UART1_SendString("\r\n");
    } else {
        UART1_SendString("Mining failed.\r\n");
    }

    while (1) {
        // Idle loop
    }
    return 0;
}
