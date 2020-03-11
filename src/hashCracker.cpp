#include "dataStructure.hpp"

#include "hashCracker.hpp"
#include "SHA256.h"


Ticker hashTicker;

void ISR_hash_signal(){
    hashCracker.flags_set(SIGNAL_HASH_TICK);
}

void TRD_hash_cracker(){
    // local data
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
                        0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
                        0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
                        0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
                        0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
                        0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    uint64_t* const key = (uint64_t*)&sequence[48];
    uint64_t* const nonce = (uint64_t*)&sequence[56];

    uint8_t hash[32];

    // start
    hashTicker.attach_us(&ISR_hash_signal,1000000);

    while(1){
        // wait for singal
        ThisThread::flags_wait_any(SIGNAL_HASH_TICK);

        // code for test 5000 nonces
        for (int i=0; i<5000; ++i){
            (*nonce)++;

            // check for key update
            if (ThisThread::flags_get() & SIGNAL_HASH_KEY_CHANGE){
                // retrive new key and clear flag
                hashKeyMutex.lock();
                *key = hashKey;
                ThisThread::flags_clear(SIGNAL_HASH_KEY_CHANGE);
                hashKeyMutex.unlock();

                pc.printf("\nSet new key %016llX\n", *key);

                // no need to reset nonce, 64 bit circular counter
            }

            // compute key and send nonce to pc
            SHA256::computeHash(hash, sequence, 64);
            if (hash[0]==0x0 && hash[1]==0x00){
                pc.printf("N%016llX\n", *nonce);
            }

        }

    }

}
