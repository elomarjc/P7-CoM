#ifndef FILTER_H
#define FILTER_H

class Filter{
  public:

  float numerator[10] =   {0,0,0,0,0,0,0,0,0,0};
  float denominator[10] = {0,0,0,0,0,0,0,0,0,0};
  
  void Initialize(float* num, float* den, int size);
  float Update(float new_value);


  private:

  float input[10] = {0,0,0,0,0,0,0,0,0,0};
  float output[10] = {0,0,0,0,0,0,0,0,0,0};

  void Advance_Array(float*array,int length);
  
  float Dot_Product(float* vec1, float* vec2, int size);

};

#endif