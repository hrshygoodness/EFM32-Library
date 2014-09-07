/*****************************************************************
 * This program takes will encrypt a firmware image for use with
 * the EFM32 AES Bootloader. 
 * 
 * The program reads a plain firmware binary file, adds a hash,
 * encrypts everything and stores the result in a new file.
 * 
 * This program uses libtomcrypt to perform the
 * actual AES encryption. libtomcrypt is released under 
 * the WTFPL license and can be downloaded from libtom.org
 * 
 * When compiling you need to link against libtomcrypt, e.g.
 * 
 *   gcc -o encrypt encrypt.c -ltomcrypt
 * 
 * The program takes two arguments: the name of the original
 * binary (the plain file) and the filename to write the
 * encrypted image to. If the file exists it will be overwritten.
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"
 * for details. Before using this software for any purpose, you must agree to the
 * terms of that agreement.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <tomcrypt.h>

/* O_BINARY is only used on Windows. 
 * Define it to zero on other systems 
 * to make the code compatible when compiling */
#ifndef O_BINARY
#define O_BINARY 0
#endif


// Header is padded to fill one XMODEM packet
#define HEADER_SIZE 0x80

#define AES_KEY_SIZE 32

#define HASH_KEY_SIZE 16

#define AES_BLOCKSIZE 16

#define XMODEM_PACKET_SIZE 128

// The AES encryption key an init vector
uint8_t encryptionKey[AES_KEY_SIZE];
uint8_t initVector[AES_BLOCKSIZE];

// A separate AES key and init vector for
// use when calculating the application hash
uint8_t hashKey[HASH_KEY_SIZE];
uint8_t hashInitVector[AES_BLOCKSIZE];

// This is the buffer for the entire encrypted file
uint8_t *buffer;

// Holds the size of the entire encrypted file
uint32_t encryptedSize;


bool readHexString(char *inputString, uint8_t *outputBytes, int numBytes)
{
  int i;

  char asciiByte[] = { 0 , 0, 0 };

  // Check if string is large enough
  if ( strlen(inputString) / 2 < numBytes ) {
    fprintf(stderr, "Error: hex string is too short!\n");
    return false;
  }

  for ( i=0; i<numBytes; i++ ) {
    strncpy(asciiByte, inputString, 2);
    if ( sscanf(asciiByte, "%x", &outputBytes[i]) < 1 ) {
      fprintf(stderr, "Error: key contains invalid characters: %s\n", asciiByte);
      return false;
    }
    inputString += 2;
  }

  return true;
}


/*****************************************************************
 * Reads the AES keys and init vectors from the config file
 *****************************************************************/
bool readKeys(char *configPath)
{
  char lvalue[128];
  char rvalue[128];

  FILE *configFile = fopen(configPath, "r");
  
  if ( configFile == NULL ) {
    fprintf(stderr, "Unable to open config file %s\n", configPath);
    return false;
  }

  bool foundAesKey = false;
  bool foundInitVector = false;
  bool foundHashKey = false;
  bool foundHashInitVector = false;


  while ( !feof(configFile) ) {
    bool success = true;

    if ( fscanf(configFile, "%s = %s", lvalue, rvalue) < 2 )
      continue;

    if ( strcmp(lvalue, "AES_KEY") == 0 ) {

      if ( !readHexString(rvalue, encryptionKey, AES_KEY_SIZE) ) {
        fprintf(stderr, "Error parsing AES key: %s\n", rvalue);
        return false;
      }
      foundAesKey = true;

    } else if ( strcmp(lvalue, "AES_INITVECTOR") == 0 ) {

      if ( !readHexString(rvalue, initVector, AES_BLOCKSIZE) ) {
        fprintf(stderr, "Error parsing initialization vector: %s\n", rvalue);
        return false;
      }
      foundInitVector = true;

    } else if ( strcmp(lvalue, "HASH_KEY") == 0 ) {

      if ( !readHexString(rvalue, hashKey, HASH_KEY_SIZE) ) {
        fprintf(stderr, "Error parsing hash key: %s\n", rvalue);
        return false;
      }
      foundHashKey = true;

    } else if ( strcmp(lvalue, "HASH_INITVECTOR") == 0 ) {

      if ( !readHexString(rvalue, hashInitVector, AES_BLOCKSIZE) ) {
        fprintf(stderr, "Error parsing hash init vector: %s\n", rvalue);
        return false;
      }
      foundHashInitVector = true;

    } else {
      fprintf(stderr, "Unknown parameter: %s\n", lvalue);
    }

  }

  fclose(configFile);

  if ( !foundAesKey ) {
    fprintf(stderr, "Missing AES key!\n");
    return false;
  } else if ( !foundInitVector ) {
    fprintf(stderr, "Missing init vector\n");
    return false;
  } else if ( !foundHashKey ) {
    fprintf(stderr, "Missing hash key\n");
    return false;
  } else if ( !foundHashInitVector ) {
    fprintf(stderr, "Missing hash init vector\n");
    return false;
  } else {

    // All keys found
    return true;
  }
}


/*****************************************************************
 * This function calculates the hash for the application. 
 * When this function returns successfully, the hash
 * is written to hashOutput.
 *
 * Returns false if the operation failed. In this case
 * the value of hashOutput should be ignored. 
 *****************************************************************/
bool calculateHash(uint8_t *startAddr, int length, uint8_t *hashOutput)
{
  // Helper variables
  int cryptError, i, j;

  // This variable will always point to the current block
  // to be decrypted
  uint8_t *curBlock;

  // Calculate the number of AES blocks
  int aesBlocks = length / AES_BLOCKSIZE; 

  // Input buffer used to hold the input block to the 
  // encryption routine
  uint8_t inputBlock[AES_BLOCKSIZE];

  // Initialize key
  symmetric_key skey;
  cryptError = aes_setup(hashKey, HASH_KEY_SIZE, 0, &skey);

  if ( cryptError != CRYPT_OK ) {
    fprintf(stderr, "Error initializing crypto library\n");
    return false;
  }

  // hashOutput will always contain the last encrypted block
  // Initialize with the init vector for the first iteration
  memcpy(hashOutput, hashInitVector, AES_BLOCKSIZE);

  // Loop over all blocks 
  for ( i=0; i<aesBlocks; i++ ) {
    
    // Get address of the current AES block
    curBlock = startAddr + i * AES_BLOCKSIZE;

    // XOR current block with previous cipher
    for ( j=0; j<AES_BLOCKSIZE; j++ ) {
      inputBlock[j] = curBlock[j] ^ hashOutput[j];
    }

    // Encrypt a block. Result is stored in hashOutput
    cryptError = aes_ecb_encrypt(inputBlock, hashOutput, &skey);

    if ( cryptError != CRYPT_OK ) {
      fprintf(stderr, "Error during hash calculation\n");
      return false;
    }
  }

  // Success
  return true;
}


/*****************************************************************
 * Reads the plain text file (the original application binary). 
 * The file is loaded into the global buffer variable. Space
 * for the buffer is allocated by this function itself.
 *****************************************************************/
bool readPlainFile(char *fileName)
{
  int plainFileSize, i;

  // Open the file
  int fileHandle = open(fileName, O_RDONLY | O_BINARY);
  if ( fileHandle < 0 ) {
    fprintf(stderr, "Error opening file %s\n", fileName);
    return false;
  }
  
  // Get the size
  plainFileSize = lseek(fileHandle, 0, SEEK_END);
  if ( plainFileSize < 0 ) {
    fprintf(stderr, "Error opening file %s\n", fileName);
    return false;
  }
  lseek(fileHandle, 0, SEEK_SET);
  
  // Calculated the total size for the encrypted output
  encryptedSize = HEADER_SIZE + plainFileSize;

  // Pad the size to matche a integer number of XMODEM packets
  // One XMODEM packet is 128 bytes
  while ( encryptedSize % XMODEM_PACKET_SIZE != 0 ) encryptedSize++;

  // Allocate memory for the encrypted file
  buffer = (uint8_t *)malloc(encryptedSize);

  // Create pointer to the where the image should be placed
  uint8_t *p = buffer + HEADER_SIZE;

  // Read the entire file
  int bytesLeft = plainFileSize;
  int bytesRead;
    
  while ( bytesLeft > 0 ) {
    bytesRead = read(fileHandle, p, bytesLeft);
    if ( bytesRead < 0 ) {
      fprintf(stderr, "Error reading file %s\n", fileName);
      return false;
    }
	
	if ( bytesRead == 0 ) {
		return false;
	}

    bytesLeft -= bytesRead;
    p += bytesRead;
  }

  close(fileHandle);

  return true;
}


/*****************************************************************
 * Calculates the appliation hash and writes the rest of 
 * the header information.
 *****************************************************************/
bool writeHeader(void)
{
  int i;

  // Get a pointer to the size field
  uint32_t *fwSize = (uint32_t *)buffer;
  
  // Get a pointer to the hash field 
  uint8_t *hash = buffer + 4;

  // Store the size of encrypted binary in header
  *fwSize = encryptedSize - HEADER_SIZE;

  // Calculate and store hash in header (words 1-4)
  if ( !calculateHash(buffer + HEADER_SIZE, encryptedSize - HEADER_SIZE, hash) ) {
    return false;
  }
  
  // Fill rest of header with random bytes
  srand(time(NULL));
  for ( i=4+AES_BLOCKSIZE; i<HEADER_SIZE; i++ ) {
	buffer[i] = rand() % 0xff;
  }

  // Print header information
  printf("=== Header Information === \n");
  printf("Size: %d\n", *fwSize);
  printf("Hash: ");
  for ( i=0; i<AES_BLOCKSIZE; i++ ) {
    printf("%.2x", hash[i]);
  }
  printf("\n");
  printf("=== End of Header Information ===\n");
  
  return true;
}

/*****************************************************************
 * Encrypt the entire firmware image (including header) with AES
 * in CBC mode. The image is encrypted in place, 
 * when this function returns the buffer array will contain the 
 * encrypted image. 
 *****************************************************************/
bool encrypt(void)
{
  // Helper variables
  int cryptError, i, j;

  // Compute number of AES blocks to encrypt
  int aesBlocks = encryptedSize / AES_BLOCKSIZE;

  // The pointer to the current block to be encrypted
  uint8_t *curBlock;

  // Pointer to the last encrypted block
  // Initialize with the init vector
  uint8_t *prevBlock = initVector;

  // Initialize key
  symmetric_key skey;
  cryptError = aes_setup(encryptionKey, AES_KEY_SIZE, 0, &skey);

  if ( cryptError != CRYPT_OK ) {
    fprintf(stderr, "Error initializing crypto library\n");
    return false;
  }

  // Loop over the entire image
  for ( i=0; i<aesBlocks; i++ ) {
    
    // Get address of the current AES block
    curBlock = buffer + i * AES_BLOCKSIZE;

    // XOR current block with the last encrypted block
    for ( j=0; j<AES_BLOCKSIZE; j++ ) {
      curBlock[j] = curBlock[j] ^ prevBlock[j];
    }

    // Encrypt block in place
    cryptError = aes_ecb_encrypt(curBlock, curBlock, &skey);

    // Store address of current block for next iteration
    prevBlock = curBlock;

    if ( cryptError != CRYPT_OK ) {
      fprintf(stderr, "Error during encryption\n");
      return false;
    }
  }

  return true;
}


/*****************************************************************
 * Writes the encrypted image to file. Returns false if writing
 * failed for any reason.
 *****************************************************************/
bool writeEncryptedFile(char *fileName)
{
  // The size of the final output file
  int outputSize = encryptedSize;

  // The number of bytes written so far
  int bytesWritten = 0;

  // Helper variable 
  int writeResult;

  // Open encrypted file for writing
  int fileHandle = open(fileName, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY);
  if ( fileHandle == -1 ) {
    fprintf(stderr, "Unable to open file %s\n", fileName);
    return false;
  }

  // Loop until entire file has been written
  while ( bytesWritten < outputSize ) {

    // Write encrypted image
    writeResult = write(fileHandle, buffer + bytesWritten, encryptedSize - bytesWritten); 

    if ( writeResult < 0 ) {
      fprintf(stderr, "Error writing to file %s\n", fileName);
      return false;
    }

    // Increase the counter
    bytesWritten += writeResult;
  }

  // Close file handle
  if ( close(fileHandle) != 0 ) {
    fprintf(stderr, "Error writing to file %s\n", fileName);
    return false;
  }

  printf("Encrypted image written to %s\n", fileName);

  return true;
}


void writeCArray(const char *format, uint8_t *values, int num, FILE *outFile)
{
  int i;
  char buf[200];
  char strValue[50];

  strcpy(buf, "");

  for ( i=0; i<num; i++ ) {
    if ( i > 0 ) {
      strcat(buf, ", ");
    }
    sprintf(strValue, "0x%.2x", values[i]);
    strcat(buf, strValue);
  }

  fprintf(outFile, format, buf);
}

    

bool writeBootloaderKeys(char *keyPath)
{
  uint32_t *p;

  FILE *keyFile = fopen(keyPath, "w");

  if ( !keyFile ) {
    fprintf(stderr, "Failed to open bootloader key file %s\n", keyPath);
    return false;
  }

  fprintf(keyFile, "#include <stdint.h>\n\n");

  writeCArray("uint8_t encryptionKey[]  = {%s};\n", 
      encryptionKey, AES_KEY_SIZE, keyFile);

  writeCArray("uint8_t initVector[]     = {%s};\n",
      initVector, AES_BLOCKSIZE, keyFile);

  writeCArray("uint8_t hashKey[]        = {%s};\n",
      hashKey, HASH_KEY_SIZE, keyFile);

  writeCArray("uint8_t hashInitVector[] = {%s};\n",
      hashInitVector, AES_BLOCKSIZE, keyFile);

  fclose(keyFile);
  return true;
}



bool file_exists(char *path)
{
  FILE *f = fopen(path, "r");

  if ( f ) {
    fclose(f);
    return true;
  } else {
    return false;
  }
}

void printUsage(char *program)
{
  fprintf(stderr,"Usage: %s <plainfile> <encryptedfile>\n"
		  "       %s --create-bootloader-keys\n", program, program, program);
}


int main(int argc, char **argv)
{

  if ( argc == 2 ) {
	if ( strcmp(argv[1], "--create-bootloader-keys") == 0 ) {
	  if ( !readKeys("keys.txt") ) {
		fprintf(stderr, "Aborted.\n");
		return 1;
	  }
	  if ( !writeBootloaderKeys("aes_keys.c") ) {
	    fprintf(stderr, "Aborted.\n");
	    return 1;
	  }
	  return 0;
    } else {
      printUsage(argv[0]);
      return 1;
    }
  } else if ( argc != 3 ) {
    printUsage(argv[0]);
    return 1;
  }

  char *plainPath = argv[1];
  char *encryptedPath = argv[2];


  if ( !readKeys("keys.txt") ) {
    fprintf(stderr, "Aborted.\n");
    return 1;
  }

  if ( !readPlainFile(plainPath) ) {
    fprintf(stderr, "Aborted.\n");
    return 1;
  }

  if ( !writeHeader() ) {
    fprintf(stderr, "Aborted.\n");
    return 1;
  }

  if ( !encrypt() ) {
    fprintf(stderr, "Aborted.\n");
    return 1;
  }

  if ( !writeEncryptedFile(encryptedPath) ) {
    fprintf(stderr, "Aborted.\n");
    return 1;
  }
  

  return 0;
}
