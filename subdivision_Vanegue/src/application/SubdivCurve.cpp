#include "SubdivCurve.h"
#include <cmath>
#include <iostream>

#include "Vector3.h"
#include "Matrix4.h"

using namespace std;
using namespace p3d;

SubdivCurve::~SubdivCurve() {
}

SubdivCurve::SubdivCurve() {
    _nbIteration=1;
    _source.clear();
    _result.clear();

}


void SubdivCurve::addPoint(const p3d::Vector3 &p) {
    _source.push_back(p);
}

void SubdivCurve::point(int i,const p3d::Vector3 &p) {
    _source[i]=p;
}


void SubdivCurve::chaikinIter(const vector<Vector3> &p) {
    /* TODO : one iteration of Chaikin : input = p, output = you must set the vector _result (vector of Vector3)
   */
    _result.clear();
    int size = p.size();

    for(int i=0; i<size-1; i+=1) {

        if(isClosed() && i == size-1){
            _result.push_back(3.0/4.0*p[i]+1.0/4.0*p[0]);
            _result.push_back(1.0/4.0*p[i]+3.0/4.0*p[0]);
        }else{
            _result.push_back(3.0/4.0*p[i]+1.0/4.0*p[i+1]);
            _result.push_back(1.0/4.0*p[i]+3.0/4.0*p[i+1]);
        }
    }

}

void SubdivCurve::dynLevinIter(const vector<Vector3> &p) {
    /* TODO : one iteration of DynLevin : input = p, output = you must set the vector _result (vector of Vector3)
   */
    _result.clear();

    int size = p.size();
    for(int i=0; i<size; i+=1) {

        if(isClosed() && i == size-1){
            _result.push_back(p[i]);
            _result.push_back(-1.0/16.0*(p[1]+p[i-1])+9.0/16.0*(p[0]+p[i]));
        }else{
            _result.push_back(p[i]);
            _result.push_back(-1.0/16.0*(p[i+2]+p[i-1])+9.0/16.0*(p[i+1]+p[i]));
        }

    }

}


void SubdivCurve::chaikin() {
    if (_source.size()<2) return;
    vector<Vector3> current;
    _result=_source;
    for(int i=0;i<_nbIteration;++i) {
        current=_result;
        chaikinIter(current);
    }
}

void SubdivCurve::dynLevin() {
    if (_source.size()<2) return;
    if (!isClosed()) return;
    vector<Vector3> current;
    _result=_source;
    for(int i=0;i<_nbIteration;++i) {
        current=_result;
        dynLevinIter(current);
    }
}


