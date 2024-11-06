#ifndef __COPYRIGHT_VERIFICATION_H__
#define __COPYRIGHT_VERIFICATION_H__


#define MAX_ENCRYPTED_DATA_LEN      (2048)        //加密后密文的最大长度


/**
 * @brief 自定义加密算法的接口类型
 * 
 */
typedef int (*ENCRYPT_CALLBACK)(char * in_buffer, int in_len, char *out_buffer, int out_len);



/**
 * @brief 通过自定义加密算法校验软件是否被拷贝
 * 
 * @param ec_func 用于传入自定义加密算法的函数指针
 * @param out_len 用于指定加密后的密文的长度
 * @param firm_ver 固件版本
 * @return int 返回校验函数是否正常运行，0：未正常运行或校验未通过，1：校验通过。
 */
extern int copyright_verification1(ENCRYPT_CALLBACK ec_func, int out_len, uint8_t firm_ver);


/**
 * @brief 通过默认加密算法和密码校验软件是否被拷贝
 * 
 * @param password 加密密码
 * @param password_len 加密密码的长度
 * @param firm_ver 固件版本
 * @return int 返回校验函数是否正常运行，0：未正常运行或校验未通过，1：校验通过。
 */
extern int copyright_verification2(char * password, int password_len, uint8_t firm_ver);



#endif //__COPYRIGHT_VERIFICATION_H__


