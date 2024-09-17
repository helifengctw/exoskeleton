#ifndef _CRC_1_H_
#define _CRC_1_H_

#include <stdint.h>
uint16_t Crc16(uint8_t *ptr, uint32_t len);

#define HIG_UINT16(a)   ( ((a)>>8) & 0xFF )     // ȡuint16�ĸ�8λ
#define LOW_UINT16(a)   ( (a) & 0xFF )          // ȡuint16�ĵ�8λ

#define HIG_UINT8(a)    ( ((a)>>4) & 0x0F )     // ȡuint8�ĸ�4λ
#define LOW_UINT8(a)    ( (a) & 0x0F )          // ȡuint8�ĵ�4λ


/* CRC16 ��ʽ�� */
static uint16_t crctalbeabs[] = { 
	0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 
	0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400 
};


/*!
 *  ��  ��: CRC16У��
 *  param1: ָ��ҪУ������ݵ�ָ��
 *  param2: ҪУ������ݵĳ���
 *  retval: У�����õ���ֵ��uint16_t ����
 * 
 *  ˵  ��: ����CRCУ��Ϊ���������ʽΪ x16+x15+x2+1(0x8005)��CRC�ĳ�ʼֵΪ0xFFFF
 */
uint16_t Crc16(uint8_t *ptr, uint32_t len) 
{
	uint16_t crc = 0xffff; 
	uint32_t i;
	uint8_t ch;
 
	for (i = 0; i < len; i++) {
		ch = *ptr++;
		crc = crctalbeabs[(ch ^ crc) & 15] ^ (crc >> 4);
		crc = crctalbeabs[((ch >> 4) ^ crc) & 15] ^ (crc >> 4);
	} 
	
	return crc;
}

#endif  /* crc.hpp */
