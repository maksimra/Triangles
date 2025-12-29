#pragma once

#include <functional>
#include <istream>
#include <iostream>

template <typename T> void get_smth_from_istream(T *smth, std::istream &input)
{
    input >> *smth;
    if (!input.good())
        throw std::runtime_error("Input error!\n");
}

template <typename T, typename C = std::less<>>
void get_positive_val_from_istream(T *value, std::istream &input, C cmp = C{})
{
    get_smth_from_istream(value, input);

    while (!cmp(0, *value))
    {
        std::cout << "You should enter positive number. Try again, please." << std::endl;

        get_smth_from_istream(value, input);
    }
}
