#include "ooqp_inter.h"

ooqp_inter::ooqp_inter()
{
}

VectorXd ooqp_inter::optimization(int var_num,int Qnnz,std::vector<int> Qrow,std::vector<int> Qcol,std::vector<double> Qvalue,int eq_con_num,int Annz, std::vector<int> Arow,
std::vector<int> Acol,std::vector<double> Avalue,std::vector<double> bvalue)
{
    int nx=var_num;
    VectorXd cv;
    cv=VectorXd::Zero( var_num );
    double  *c;
    c=cv.data();
    double xupp[var_num],xlow[var_num];
    char ixupp[var_num],ixlow[var_num];
    for(int i=0;i<var_num;i++)
    {
        xupp[i]=0;
        xlow[i]=0;
        ixupp[i]=0;
        ixlow[i]=0;
    }

    ///Q matrix
    int nnzQ=Qnnz;
    int *irowQ,*jcolQ;
    double *dQ;
    irowQ=Qrow.data();
    jcolQ=Qcol.data();
    dQ=Qvalue.data();

    /////equality contraintsint
    int my         = eq_con_num;
    int nnzA       = Annz;

    int *irowA=new int[Arow.size()];
    int *jcolA=new int[Acol.size()];
    double *dA=new double[Avalue.size()];
    double *b = new double[bvalue.size()];
    irowA=Arow.data();
    jcolA=Acol.data();
    dA=Avalue.data();
    b =bvalue.data();
    for(int j=0;j<Avalue.size();j++)
        std::cout<<dA[j]<<std::endl;

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

    int ierr = s->solve(prob,vars, resid);
    if( ierr == 0 ) {
        cout.precision(4);
        cout << "Solution: \n";
        vars->x->writefToStream( cout, "x[%{index}] = %{value}" );
    } else {
        cout << "Could not solve the problem.\n";
    }
    double var_arr[vars->x->length()];
    vars->x->copyIntoArray(&var_arr[0]);
    int size=vars->x->length();
    VectorXd var_vec;
    var_vec.resize(size);
    for(int i=0;i<size;i++)
        var_vec(i)=var_arr[i];
    return var_vec;
}

