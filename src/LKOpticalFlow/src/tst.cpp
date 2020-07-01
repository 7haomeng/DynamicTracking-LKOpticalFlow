#include <bits/stdc++.h> 
using namespace std; 
  
int main() 
{ 
    // initialising the vector 
    vector<int> vec1 = { 10, 20, 30, 40 }; 
    vector<int> vec3 = { 10, 20, 30, 40 ,50}; 
    vector<int> vec2; 
  
    // inserts at the beginning of vec2 
    vec2.insert(vec2.end(), vec1.begin(), vec1.end()); 
    vec2.insert(vec2.begin(), vec3.begin(), vec3.end());
    cout << vec2.size() << endl;
    cout << vec2.capacity() << endl;
    cout << "The vector2 elements are: "; 
    for (auto it = vec2.begin(); it != vec2.end(); ++it) 
        cout << *it << " "; 
        cout <<endl;
        cout << *(vec2.end()-1) << endl;
        cout <<endl;
  
    return 0; 
}