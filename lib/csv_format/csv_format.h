/**
 * @file csv_format.h
 * @author Scansati Team 2021
 * @date 13 May 2021
 * @brief Facility functions to format data in CSV
 */

#ifndef __CSV_FORMAT__
#define __CSV_FORMAT__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @param buf target buffer; data will be added here
 * @param ... title1, fields_num1, title2, fields_num2
 * @param ... the last parameter must be NULL
 */
void csv_heading(char *buf, ...);

/**
 * @param buf target buffer; data will be added here
 * @param ... value1, value2, value3...
 * @param ... the last parameter must be NULL
 */
void csv_heading(char *buf, ...);

#ifdef __cplusplus
}
#endif

#endif  // __CSV_FORMAT__