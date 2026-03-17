#pragma once

#include <functional>
#include <iostream>
#include <istream>

template <typename T> void getSmthFromIstream(T *smth, std::istream &input) {
  input >> *smth;
  if (!input.good())
    throw std::runtime_error("Input error!\n");
}

template <typename T, typename C = std::less<>>
void getPositiveValFromIstream(T *value, std::istream &input, C cmp = C{}) {
  getSmthFromIstream(value, input);

  while (!cmp(0, *value)) {
    std::cout << "You should enter positive number. Try again, please."
              << std::endl;

    getSmthFromIstream(value, input);
  }
}
