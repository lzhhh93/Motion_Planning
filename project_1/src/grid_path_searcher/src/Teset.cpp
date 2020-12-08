#include<iostream>
#include<Eigen/Eigen>

int main(){
   std::map<int, int> o;
   std::map<int,int>::iterator it;
   o.insert( std::make_pair(45,23) );
   o.insert( std::make_pair(21,24) );
   o.insert( std::make_pair(35,25) );
   o.insert( std::make_pair(10,35) );
   for( it = o.begin() ; it != o.end() ;it++)
   std::cout<< it->first << "--" << it->second << std::endl;
   //it = o.begin();
   o.erase(o.begin());
   if( o.begin() == o.end() )
   std::cout<< "None!" << std::endl;
   else
   std::cout<< "-------" << std::endl;
   {
       for( it = o.begin() ; it != o.end() ;it++)
   std::cout<< it->first << "--" << it->second << std::endl;
  
   }
   std::cout<< "-------" << std::endl;
   it = o.end();
   std::cout<< it->first << std::endl;
   it = o.find(it->first);
   //o.erase(it);
    //it = o.end();

   //std::cout<< it->first << std::endl;
  std::cin.get();
}

