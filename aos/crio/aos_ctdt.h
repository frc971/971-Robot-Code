#ifndef AOS_CTDT_H_
#define AOS_CTDT_H_

// This function will call any function that starts with aos_init_function_*.
// It will assume that these functions have the signature
// 'extern "C" aos_init_function_whatever(void);'
// The aos_ctdt.c/o files are generated at compile time (like ctdt.c/o).
#ifdef __cplusplus
extern "C" {
#endif
void aos_call_init_functions();
#ifdef __cplusplus
}
#endif

#endif

