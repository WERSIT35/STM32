#include <stdio.h>

void return0(int number1, int number2){
    int sum = number1 + number2;
	printf("%d\n", sum);
}

int main(){
    printf("Hello World\n");


    return0(15, 32);
    return 0;
}