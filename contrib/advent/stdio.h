#ifndef STDIO_H
#define STDIO_H

#include <stddef.h>

#define EOF -1

typedef void FILE;
extern FILE *stdin, *stdout;
extern int printf(const char *, ...);
extern int fflush(FILE *);
extern char *fgets(char *, int, FILE *);
extern int getchar(void);
extern int putchar(int);
extern int puts(const char *);

#endif /* STDIO_H */
