#include <Arduino.h>

template <typename T>
bool arrayContains(const T *array, int size, T element)
{
    for (int i = 0; i < size; i++)
    {
        if (element == array[i])
        {
            return true;
        }
    }

    return false;
}