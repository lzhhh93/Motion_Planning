#include "ooqp_test_node.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ooqp_test_node");
    ros::NodeHandle nh("~");
    int usage_ok = 1, quiet = 0;
    if( argc > 2 ) usage_ok = 0;
    if( argc == 2 ) {
        if( 0 == strcmp( "--quiet", argv[1] ) ) {
            quiet = 1;
        } else {
            usage_ok = 0;
        }
    }
    if( !usage_ok ) {
        cerr << "Usage: " <<  argv[0] << " [ --quiet ]\n";
        return 1;
    }
    std::vector<double> Q;
    Q.push_back(36);
    Q.push_back(72);
    Q.push_back(192);
    Q.push_back(120);
    Q.push_back(360);
    Q.push_back(720);
//    MatrixXd Q = MatrixXd::Zero(1, 6);
//    Q<<36,72,192,120,360,720;
//    VectorXd Qvalue1(3);
//    Qvalue1<<36,72,192;
//    VectorXd Qvalue;
//    Qvalue.resize(3);
//    Qvalue<<36,72,192;
//    VectorXd Qvalue2(3);
//    Qvalue2<<120,360,720;
//    Qvalue=Qvalue,Qvalue2;
//    std::cout<<Qvalue<<std::endl;
//    Qvalue1.size();



    const int nx   = 6;
    double    c[]  = { 0,0,0,0,0,0};

    ///boundary of variable
    double  xupp[] = { 0,0,0,0,0,0};
    char   ixupp[] = { 0,0,0,0,0,0};

    double  xlow[] = {0,0,0,0,0,0};
    char   ixlow[] = {0,0,0,0,0,0};

    ///Q matrix
    const int nnzQ = 6;

    int    irowQ[] = {  3,   4,   4,   5,  5,  5};
    int    jcolQ[] = {  3,   3,   4,   3,  4,  5};

//    double *dQ=Qvalue.data();
    double *dQ=new double[Q.size()];
    dQ=&Q[0];
//    if(!Q.empty())
//        memcpy(dQ,&Q[0],Q.size()*sizeof(double));



    /////equality contraintsint
    int my         = 6;
    double b[]     = {0,0,0,1,1,1};
    int nnzA       = 18;
    int irowA[]    = {0,1,2,3,3,3,3,3,3,4,4,4,4,4,5,5,5,5};
    int jcolA[]    = {0,1,2,0,1,2,3,4,5,1,2,3,4,5,2,3,4,5};
    double dA[]    = {1,1,2,1,1,1,1,1,1,1,2,3,4,5,2,6,12,20};


    ///inequality contraints
    const int mz   = 0;
    double clow[]  = {};
    char  iclow[]  = {};

    double cupp[]  = {};
    char  icupp[]  = {};

    const int nnzC = 0;
    int   irowC[]  = {};
    int   jcolC[]  = {};
    double   dC[]  = {};

    QpGenSparseMa27 * qp
            = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );

    QpGenData      * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
                c,      irowQ,  nnzQ,   jcolQ,  dQ,
                xlow,   ixlow,  xupp,   ixupp,
                irowA,  nnzA,   jcolA,  dA,     b,
                irowC,  nnzC,   jcolC,  dC,
                clow,   iclow,  cupp,   icupp );

    QpGenVars      * vars
            = (QpGenVars *) qp->makeVariables( prob );
    QpGenResiduals * resid
            = (QpGenResiduals *) qp->makeResiduals( prob );

    GondzioSolver  * s     = new GondzioSolver( qp, prob );

    if( !quiet ) s->monitorSelf();
    int ierr = s->solve(prob,vars, resid);

    if( ierr == 0 ) {
        cout.precision(4);
        cout << "Solution: \n";
        vars->x->writefToStream( cout, "x[%{index}] = %{value}" );
    } else {
        cout << "Could not solve the problem.\n";
    }
    return 0;


}
