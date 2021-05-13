/**
 * @file csv_format.c
 * @author Scansati Team 2021
 * @date 13 May 2021
 * @brief Facility functions to format data in CSV
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define SEPARATOR ','

char *strup(const char *str) {
    char *rslt = malloc(strlen(str)+1);
    int i = 0;
    for (;i<strlen(str); i++) *(rslt+i) = (unsigned char) toupper(*(str+i));
    *(rslt+i) = 0;
    return rslt;
}

void csv_heading(char *buf, ...) {
    va_list pargs;
    va_start(pargs, buf);
    char *arg;
    int buf_len = 0;

    *buf = 0;
    for (;;) {
        arg = va_arg(pargs, char *);  // TODO: add fields_num arg (title, fields_num, title2, fields_num2...)
        if (arg == NULL)
            break;
        buf_len += sprintf(buf+buf_len, "%s%c", strup(arg), SEPARATOR);
    }
    sprintf(buf+buf_len, "\n");
}

void csv_content(char *buf, ...) {
    va_list pargs;
    va_start(pargs, buf);
    char *arg;
    int buf_len = 0;

    *buf = 0;
    for (;;) {
        arg = va_arg(pargs, char *);
        if (arg == NULL)
            break;
        buf_len += sprintf(buf+buf_len, "%s%c", arg, SEPARATOR);
    }
    sprintf(buf+buf_len, "\n");
}
