/*

[Input SCC format]
vertex indices start from 0



---


N  = number of nodes in the SCC


v1 OutDeg(v1) <indices of the vertices adj to v1>


v2 OutDeg(v2) <indices of the vertices adj to v2>


...


vN OutDeg(vN) <indices of the vertices adj to vN>


----





e.g for 3-vertex complete directed graph the input is





3


0 2 1 2


1 2 0 2


2 2 0 1





[Usage] ./a.out < input.txt


*/





#include <algorithm>


#include <iostream>


#include <vector>


#include <list>





using namespace std;





typedef vector< list<size_t> > AdjListT;


AdjListT A, B;
vector<size_t> node_stack;


vector<size_t> blocked;


size_t s;





void UNBLOCK(size_t);


bool CIRCUIT(size_t);


void OutputCycle(vector<size_t>&);





int main(void)


{


    size_t N, copy_of_N;


    cin >> N;


    copy_of_N = N;


    A.resize(N);


    blocked.resize(N);


    B.resize(N);





    while (N--) {


        size_t u, v, out_degree;





        cin >> u >> out_degree;


        while (out_degree--) {


            cin >> v;


            A.at(u).push_back(v);


        }


    }


    N = copy_of_N;


    for (size_t i = 0; i < N; ++i)


        blocked[i] = false;


    s = 0;


    while (s < N) {

     CIRCUIT(s);





        A[s].clear();


        for (size_t i = 0; i < N; ++i)


            A[i].remove(s);


        for (size_t i = 0; i < N; ++i) {


            blocked[i] = false;


            B[i].clear();


        }





        ++s;


    }


    return 0;


}





bool CIRCUIT(size_t v)


{


    bool f = false;


    node_stack.push_back(v);


    blocked[v] = true;


    for (list<size_t>::iterator iw = A[v].begin(); iw != A[v].end(); ++iw) {


        size_t w = *iw;


        if (w == s) {


            OutputCycle(node_stack);


            f = true;


        } else if (!blocked[w]) {


              if (CIRCUIT(w))


                  f = true;


        }


    }


    if (f)

      UNBLOCK(v);


    else


       for (list<size_t>::iterator iw = A[v].begin(); iw != A[v].end(); ++iw) {


           size_t w = *iw;


           if (find(B[w].begin(), B[w].end(), v) == B[w].end())


               B[w].push_back(v);


        }





    node_stack.pop_back();


    return f;


}





void UNBLOCK(size_t u)


{


    blocked[u] = false;


    for (list<size_t>::iterator iw = B[u].begin(); iw != B[u].end();) {


        size_t w = *iw;


        iw = B[u].erase(iw);


        if (blocked[w])


            UNBLOCK(w);


    }


    return;


}





void OutputCycle(vector<size_t>& node_stack)


{


    for (vector<size_t>::iterator i = node_stack.begin(); i != node_stack.end(); ++i)


        cout << *i << " ";


    cout << endl;


    return;


}

