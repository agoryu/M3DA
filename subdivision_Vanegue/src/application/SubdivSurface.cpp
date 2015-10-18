#include "SubdivSurface.h"
#include "GLTool.h"

using namespace p3d;
using namespace std;

SubdivSurface::~SubdivSurface() {

}


SubdivSurface::SubdivSurface() {

}


void SubdivSurface::input(p3d::Mesh *m) {
    _input=m;
}

void SubdivSurface::source(p3d::Mesh *m) {
    _source=m;
}


void SubdivSurface::source(const std::string &filename) {
    delete _source;
    _source=new Mesh();
    _source->readInit(filename,false);
    _source->requestInitDraw();
}


/** returns the edge index of [v1,v2] (-1 if not found)
 *
 * @pre : vector _edgeForVertex must be intialized before
 */
int SubdivSurface::findEdge(int v1,int v2) {
    ///
    vector<int> &edgeV1=_edgeOfVertex[v1];
    for(unsigned int i=0;i<edgeV1.size();++i) {
        int b=_edge[edgeV1[i]]._b;
        if (b==v2) return edgeV1[i];
    }
    vector<int> &edgeV2=_edgeOfVertex[v2];
    for(unsigned int i=0;i<edgeV2.size();++i) {
        int b=_edge[edgeV2[i]]._b;
        if (b==v1) return edgeV2[i];
    }
    return -1;
}



void SubdivSurface::prepare() {
    if (!_input) return;
    /// compute edge :
    ///
    int v1,v2;
    _edgeOfVertex.clear();
    _edge.clear();
    _edgeOfVertex.resize(_input->nbPosition());
    for(unsigned int i=0;i<_input->nbFace();++i) { // i is the face (index)
        v1=_input->positionIndex(i,-1); // the vertex before the first one in face i (i.e. the last one)
        for(unsigned int j=0;j<_input->nbVertex(i);++j) { // j is the j-th vertex in the face i
            v2=_input->positionIndex(i,j);
            int foundEdge=findEdge(v1,v2);
            if (foundEdge!=-1) { // the edge already exists
                _edge[foundEdge]._right=i; // if already exists, the _left face is already set
            }
            else { // create the edge [v1,v2]
                Edge e;
                e._a=v1;
                e._b=v2;
                e._left=i; // the current face is put on _left (preserve the orientation of the mesh)
                e._right=-1; // will be set if the same edge is encountered after
                _edge.push_back(e);
                _edgeOfVertex[v1].push_back(_edge.size()-1);
                _edgeOfVertex[v2].push_back(_edge.size()-1);
            }
            v1=v2; // next
        }
    }
}



void SubdivSurface::computePointFace() {
    /* TODO : compute all point face (set the vector _pointFace : _pointFace[i] for the i-th face of _input).
   * - input = Mesh *_input
   * - _input->nbFace()
   * - _input->nbVertex(i) : number of vertices of i-th face
   * - _input->positionMesh(i,j) : the position of the j-th vertex of the i-th face
   */
    _pointFace.clear();

    int nbFace = _input->nbFace();
    for(int i=0; i<nbFace; i++) {

        int nbVertex = _input->nbVertex(i);
        _pointFace.push_back(Vector3(0,0,0));
        for(int j=0; j<nbVertex; j++) {
            _pointFace[i] += _input->positionMesh(i,j);
        }
        _pointFace[i] /= nbVertex;
    }
}

void SubdivSurface::computePointEdge() {
    /* TODO : compute all point face (set the vector _pointEdge : _pointEdge[i] for the i-th edge).
   * input = Mesh *_input
   * _edge[i]._a, _edge[i]._b : give the index of the two positions of i-th edge (._a and ._b are indexes for _input)
   * _input->position(i) : the i-th position of input (as referred by ._a and ._b).
   * _edge[i]._left, _edge[i]._right : indexes of the two faces incident to i-th face
   * - _pointFace[i] : should give the point Face of the i-th face.
   */
    _pointEdge.clear();

    int size = _edge.size();
    for(int i=0; i<size; i++) {
        Vector3 v1 = _input->positionMesh(_edge[i]._a);
        Vector3 v2 = _input->positionMesh(_edge[i]._b);
        Vector3 f1 = _pointFace[_edge[i]._left];
        Vector3 f2 = _pointFace[_edge[i]._right];
        _pointEdge.push_back((v1+v2+f1+f2)/4);
    }
}


void SubdivSurface::computePointVertex() {
    /* TODO : compute all point vertex (set the vector _pointVertex : _pointVertex[i] for the i-th position).
   * input = Mesh *_input
   * - _edgeOfVertex[i][j] : gives the index (for the vector _edge) of the j-th edge of the i-th vertex
   */
    _pointVertex.clear();

    double nbVertex = _edgeOfVertex.size();

    for(int i=0; i<nbVertex; i++) {
        Vector3 vi = _input->positionMesh(i);
        double nbEdge = _edgeOfVertex[i].size();

        _pointVertex.push_back((nbEdge - 2.0) / nbEdge * vi);

        Vector3 sumEdge(0.0, 0.0, 0.0);
        Vector3 sumFace(0.0, 0.0, 0.0);

        for(int j=0; j<nbEdge; j++) {
            sumEdge += _pointEdge[_edgeOfVertex[i][j]];
            if(_edge[_edgeOfVertex[i][j]]._a == i)
                sumFace += _pointFace[_edge[_edgeOfVertex[i][j]]._left];
            else
                sumFace += _pointFace[_edge[_edgeOfVertex[i][j]]._right];
        }

        _pointVertex[i] += 1.0/(nbEdge*nbEdge) * sumEdge + 1.0/(nbEdge*nbEdge) * sumFace;
    }
}

int SubdivSurface::findNextEdge(int i,int j) {

    return -1; // happens for a boundary edge
}

void SubdivSurface::buildMesh() {
    Mesh *m=new Mesh();
    /* TODO : build the new mesh
   * - m->addPositionMesh(aVector3) to add a vertex
   * - m->addFaceMesh({v1,v2,v3,...}) to add a face : caution : v1,v2,v3,... are indexes (int) of the positions of m
   * - caution with the indexes (indexes of m are not the same that the ones for _input : track them).
   *
   */

    int sizeV = _pointVertex.size();
    for(int i=0; i<sizeV; i++) {
       m->addPositionMesh(_pointVertex[i]);
    }

    int sizeF = _pointFace.size();
    for(int i=0; i<sizeF; i++) {
       m->addPositionMesh(_pointFace[i]);
    }

    int sizeE = _pointEdge.size();
    for(int i=0; i<sizeE; i++) {
       m->addPositionMesh(_pointEdge[i]);
    }

    for(int i=0; i<sizeV; i++) {

        double nbIncidentFace = _edgeOfVertex[i].size();
        for(int j=0; j<nbIncidentFace; j++) {
            int ip = i;
            int ie1 = sizeV + sizeF + _edgeOfVertex[i][j];
            int ifa = 0;

            if(ip == _edge[_edgeOfVertex[i][j]]._a) {
                ifa = sizeV + _edge[_edgeOfVertex[i][j]]._left;
            } else {
                ifa = sizeV + _edge[_edgeOfVertex[i][j]]._right;
            }

            int ie2 = 0;
            for(int k=0; k<nbIncidentFace; k++) {

                if(ip == _edge[_edgeOfVertex[i][j]]._a) {

                    if(ip == _edge[_edgeOfVertex[i][k]]._a) {
                        if(_edge[_edgeOfVertex[i][j]]._left == _edge[_edgeOfVertex[i][k]]._right) {
                            ie2 = sizeV + sizeF + _edgeOfVertex[i][k];
                        }
                    }

                    if(ip == _edge[_edgeOfVertex[i][k]]._b) {
                        if(_edge[_edgeOfVertex[i][j]]._left == _edge[_edgeOfVertex[i][k]]._left) {
                            ie2 = sizeV + sizeF + _edgeOfVertex[i][k];
                        }
                    }


                } else {
                    if(ip == _edge[_edgeOfVertex[i][k]]._a) {
                        if(_edge[_edgeOfVertex[i][j]]._right == _edge[_edgeOfVertex[i][k]]._right) {
                            ie2 = sizeV + sizeF + _edgeOfVertex[i][k];
                        }
                    }

                    if(ip == _edge[_edgeOfVertex[i][k]]._b) {
                        if(_edge[_edgeOfVertex[i][j]]._right == _edge[_edgeOfVertex[i][k]]._left) {
                            ie2 = sizeV + sizeF + _edgeOfVertex[i][k];
                        }
                    }
                }

            }

            m->addFaceMesh({ip, ie1, ifa, ie2});
        }
    }

    /* end TODO */


    _result=m;
    _result->computeNormal();
    _result->computeTexCoord();
}


void SubdivSurface::catmullClarkIter() {
    prepare();
    computePointFace();
    computePointEdge();
    computePointVertex();
    buildMesh();
}

void SubdivSurface::catmullClark() {
    delete _result;
    _pointVertex.clear();
    _pointEdge.clear();
    _pointFace.clear();
    _result=_source->clone();
    for(int i=0;i<_nbIteration;++i) {
        _input=_result;
        catmullClarkIter();
        delete _input;
    }
}




void SubdivSurface::drawTest() {
    glPointSize(10);


    p3d::ambientColor=Vector4(1,0,0,1);
    p3d::shaderVertexAmbient();
    if (_pointVertex.size()>0) p3d::drawPoints(_pointVertex);


    p3d::ambientColor=Vector4(0,1,0,1);
    p3d::shaderVertexAmbient();
    if (_pointEdge.size()>0) p3d::drawPoints(_pointEdge);


    p3d::ambientColor=Vector4(0,0,1,1);
    p3d::shaderVertexAmbient();
    if (_pointFace.size()>0) p3d::drawPoints(_pointFace);
}





