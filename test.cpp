#include <iostream>
#include <chrono>
#include <windows.h>

int main(){
    auto start = std::chrono::system_clock::now();
    Sleep(1000);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Elapsed time: " << elapsed_seconds.count() << " seconds\n";
    return 0;
}