#include <stdio.h>

int main(void)
{
    long long a[60];
    long long b[60];
    long long c[60];
    long long d[60];

    /* i = 0 的初值 */
    a[0] = 0;
    b[0] = 1;

    /* i = 0 时 c[0], d[0] 的计算 */
    c[0] = a[0];
    d[0] = b[0];

    for (int i = 1; i <= 59; i++) {

        /* 对应滚动更新 a, b */
        a[i] = a[i - 1] + i;
        b[i] = b[i - 1] + 3LL * i;

        /* 计算 c[i] */
        if (i <= 19) {
            c[i] = a[i];
        } else if (i <= 39) {
            c[i] = a[i] + b[i];
        } else {
            c[i] = a[i] * b[i];
        }

        /* 计算 d[i] */
        if (i <= 19) {
            d[i] = b[i];
        } else if (i <= 39) {
            d[i] = a[i] * c[i];
        } else {
            d[i] = c[i] * b[i];
        }
    }

    printf("a[59] = %lld\n", a[59]);
    printf("b[59] = %lld\n", b[59]);
    printf("c[59] = %lld\n", c[59]);
    printf("d[59] = %lld\n", d[59]);

    return 0;
}

