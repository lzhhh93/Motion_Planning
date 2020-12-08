#include<iostream>
#include<map>
#include<vector>

struct num{

int a,b;

};

int main()
{
  num* a;
  num b;
  a = &b;
  int r =2;
  int x = a->a+r; 
  std::cout<< b <<std::endl;
}

