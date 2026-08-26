/* Puente C++ -> C: fall_model.h (generado por micromlgen) es una clase
 * C++, pero el resto del firmware es C. Esta funcion con enlace "C" es
 * la unica puerta de entrada que myosa_field_main.c necesita conocer. */
#include <cstdint>  /* fall_model.h usa uint8_t pero solo incluye <cstdarg> */
#include "fall_model.h"

extern "C" int fall_model_predict(const float *x)
{
    static Eloquent::ML::Port::RandomForest clf;
    return clf.predict(const_cast<float *>(x));
}
