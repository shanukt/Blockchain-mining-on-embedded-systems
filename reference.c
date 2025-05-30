#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <openssl/sha.h>

#define MAX_NONCE 100000000000ULL

// Function to compute SHA256 hash of the input text.
// The returned string is dynamically allocated and should be freed by the caller.
char* SHA256_hash(const char* text) {
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256((unsigned char*)text, strlen(text), digest);

    // Each byte gives two hex characters, plus one for the null terminator.
    char* output = malloc(2 * SHA256_DIGEST_LENGTH + 1);
    if(output == NULL) {
        fprintf(stderr, "Memory allocation error\n");
        exit(EXIT_FAILURE);
    }
    
    for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
         sprintf(output + (i * 2), "%02x", digest[i]);
    }
    output[2 * SHA256_DIGEST_LENGTH] = '\0';
    return output;
}

// Check if the hash string starts with the required number of zeros.
int starts_with(const char* str, int prefix_zeros) {
    for (int i = 0; i < prefix_zeros; i++) {
        if (str[i] != '0')
            return 0;
    }
    return 1;
}

// The mining function. It concatenates the block number, transactions,
// previous hash, and nonce into a string, then computes its SHA256 hash.
// It continues until it finds a hash that starts with the given number of zeros.
char* mine(int block_number, const char* transactions, const char* previous_hash, int prefix_zeros) {
    // Create a string containing the required number of '0's.
    char prefix_str[prefix_zeros + 1];
    memset(prefix_str, '0', prefix_zeros);
    prefix_str[prefix_zeros] = '\0';
    
    char text[1024];  // buffer to hold the string to be hashed
    char *hash_str = NULL;
    
    for (unsigned long long nonce = 0; nonce < MAX_NONCE; nonce++) {
        // Prepare the string with block data and nonce.
        snprintf(text, sizeof(text), "%d%s%s%llu", block_number, transactions, previous_hash, nonce);
        hash_str = SHA256_hash(text);
        
        if (starts_with(hash_str, prefix_zeros)) {
            printf("Successfully mined bitcoins with nonce value: %llu\n", nonce);
            return hash_str;  // Caller is responsible for freeing this memory.
        }
        
        // If not matching, free the hash string and try the next nonce.
        free(hash_str);
    }
    
    fprintf(stderr, "Couldn't find correct hash after trying %llu times\n", MAX_NONCE);
    exit(EXIT_FAILURE);
}

int main() {
    // Demo Bitcoin Transaction
    const char* transactions = "George->Brwon->100,Robin->Russel->300";
    int difficulty = 5; // Higher values increase the difficulty.
    int block_number = 5;
    const char* previous_hash = "0000000xa036944e29568d0cff17edbe038f81208fecf9a66be9a2b8321c6ec7";
    
    printf("Start mining...\n");
    clock_t start = clock();
    char* new_hash = mine(block_number, transactions, previous_hash, difficulty);
    clock_t end = clock();
    
    double time_taken = (double)(end - start) / CLOCKS_PER_SEC;
    printf("End mining. Mining took: %f seconds\n", time_taken);
    printf("Hash: %s\n", new_hash);
    
    free(new_hash);
    return 0;
}
