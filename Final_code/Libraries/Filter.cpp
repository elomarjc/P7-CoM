#include "Filter.h"


void Filter::Initialize(float* num, float* den, int size){
    for(int i = 0; i < size; i++){
      numerator[i] = num[i];
      denominator[i] = den[i];
    }
  }

float Filter::Update(float new_value){
  static int first_time = true;
    // Right now butter 3rd order, 15Hz cut-off wiht Fs = 20Hz

  Advance_Array(input,10);
  Advance_Array(output,10);

  input[0] = new_value;
  output[0]  = Dot_Product(numerator,input,10) + Dot_Product(denominator,output,10);

  if(first_time){
    first_time = false;
    return new_value;
  }
  return output[0];
}



float Filter::Dot_Product(float* vec1, float* vec2, int size){
  float sum = 0;
  for(int i = 0; i < size;i++){
    sum+= vec1[i]*vec2[i];
  }
  return sum;
}

void Filter::Advance_Array(float*array,int length){
  for(int i = length-1; i >0; i--){
    array[i] = array[i-1];
  }
  array[0] = 0;
}
