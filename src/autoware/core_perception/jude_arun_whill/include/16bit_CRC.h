//Created by Shunichiro Sugiyama 2018
// cf. Wikipedia 「巡回冗長検査」
//16bit CRC for IMU "CSM-MG100"

uint16_t crc_table[128];

/* 事前にこの関数を実行しておくこと */
void make_crc_table(void){
    for(uint16_t i=0; i<128; i++){
        uint16_t c = i << 8;
        for( int j=0; j<8; j++){
            c = ( c << 1) ^ ( ( c & 0x8000) ? 0x1021 : 0);
        }
        crc_table[i] = c;
    }
}

uint16_t crc16ccitt(uint8_t *buf, int len){
    uint16_t c = 0xffff;
    for (int i = 0; i < len; i++)    {
        c = (c << 8) ^ crc_table[((c >> 8) ^ buf[i]) & 0x80];
    }
    return c;
}
