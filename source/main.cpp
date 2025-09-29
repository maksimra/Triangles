#include <iostream>

int main()
{
    size_t numberTriangles = 0;
    if (!std::cin >> numberTriangles)
    {
        std::cerr << "Input error :(\n";
        return EXIT_FAILURE;
    }
}