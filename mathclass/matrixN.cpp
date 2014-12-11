#include "matrixN.h"
#include "vectorN.h"

#include <cassert>

using std::istream;
using std::ostream;
using std::cerr;
using std::endl;

using math::matrixN;
using math::vectorN;

math::matrixN::matrixN()
{
    on = n = m = 0;
}

math::matrixN::matrixN( int x, int y )
{
    on = n = x;
    m = y;
    v = new vectorN[n];

    for( int i=0; i<n; i++ )
        v[i].setSize( m );
}

math::matrixN::matrixN( int x, int y, vectorN *a )
{
    on = n = x;
    m = y;
    v = a;
}

math::matrixN::matrixN( const matrixN& a )
{
	on = n = m = 0;

	matrixN &c = ( *this );
	c.setSize( a.row(), a.column() );

	for( int i=0; i < a.row(); i++ )
	{
		for( int j=0; j < a.column(); j++ )
		{
			c[i][j] = a[i][j];
		}
	}
}

math::matrixN& math::matrixN::operator =( const matrixN& a )
{
	matrixN &c = ( *this );
	c.setSize( a.row(), a.column() );

	for( int i=0; i < a.row(); i++ )
	{
		for( int j=0; j < a.column(); j++ )
		{
			c[i][j] = a[i][j];
		}
	}
	return c;
}

math::matrixN::~matrixN()
{
    if ( on>0 && m>0 ) delete[] v;
}

void
math::matrixN::getValue( double **d ) const
{
	for( int i=0; i<n; i++ )
	for( int j=0; j<m; j++ )
		v[i][j] = d[i][j];
}

double
math::matrixN::getValue( int row, int column ) const
{
	return v[row][column];
}

vectorN&
math::matrixN::operator[]( int i ) const
{
	assert(i>=0 && n>i); 
	return v[i]; 
};

void
math::matrixN::setValue( double **d )
{
	for( int i=0; i<n; i++ )
	for( int j=0; j<m; j++ )
		d[i][j] = v[i][j];
}

void
math::matrixN::setValue( int row, int column, double value )
{
	this->v[row].setValue( column, value );
}

void
math::matrixN::setRow( int x, const vectorN& vec )
{
	for( int i=0; i<m; i++ )
		v[x][i] = vec[i];
}

void
math::matrixN::setColumn( int x, const vectorN& vec )
{
	for (int i=0; i<n; i++)
		v[i][x] = vec[i];
}

void
math::matrixN::setSize( int x, int y )
{
	if( on<x )
	{
		if ( on>0 ) delete[] v;
		v = new vectorN[x];
		on=x;
		for( int i=0; i<x; i++ )
			v[i].setSize( y );
	}
	else if( m<y )
		for( int i=0; i<on; i++ )
			v[i].setSize( y );
	n = x;
	m = y;
}

matrixN&
math::matrixN::assign( matrixN const& a )
{
    matrixN &c = (*this);
    c.setSize( a.row(), a.column() );

    for( int i=0; i<a.row(); i++ )
    for( int j=0; j<a.column(); j++ )
        c[i][j] = a[i][j];

    return c;
}

matrixN&
math::matrixN::operator+=( matrixN const& a )
{
    matrixN &c = (*this);
    c.setSize( a.row(), a.column() );

    for( int i=0; i<a.row(); i++ )
    for( int j=0; j<a.column(); j++ )
        c[i][j] += a[i][j];

    return c;
}

matrixN&
math::matrixN::operator-=( matrixN const& a )
{
    matrixN &c = (*this);
    c.setSize( a.row(), a.column() );

    for( int i=0; i<a.row(); i++ )
    for( int j=0; j<a.column(); j++ )
        c[i][j] -= a[i][j];

    return c;
}

matrixN&
math::matrixN::operator*=( double a )
{
    matrixN &c = (*this);

    for( int i=0; i<c.row(); i++ )
    for( int j=0; j<c.column(); j++ )
        c[i][j] *= a;

    return c;
}

matrixN&
math::matrixN::operator/=( double a )
{
    matrixN &c = (*this);

    for( int i=0; i<c.row(); i++ )
    for( int j=0; j<c.column(); j++ )
        c[i][j] /= a;

    return c;
}

matrixN&
math::matrixN::mult( matrixN const& a, matrixN const& b )
{
    matrixN &c = (*this);
    assert( a.column()==b.row() );
    c.setSize( a.row(), b.column() );

    for( int i=0; i<a.row(); i++ )
    for( int j=0; j<b.column(); j++ )
    {
        c[i][j] = 0;
        for( int k=0; k<a.column(); k++ )
            c[i][j] += a[i][k] * b[k][j];
    }

    return c;
}

matrixN&
math::matrixN::transpose( matrixN const& a )
{
    matrixN &c = (*this);
    c.setSize( a.column(), a.row() );

    for( int i=0; i<a.row(); i++ )
    for( int j=0; j<a.column(); j++ )
        c[j][i] = a[i][j];

    return c;
}

bool
math::matrixN::LUdecompose( int* index )
{
	assert( this->row() == this->column() );

	int n = this->row();
	int i, j, k, imax;
	double big, dum, sum, temp;

	static vectorN vv; vv.setSize( n );
	matrixN &a = (*this);

	for ( i=0; i<n; i++ )
	{
		big = 0.0f;
		for ( j=0; j<n; j++ )
			if ((temp = fabs(a[i][j])) > big)
				big = temp;

		if (big == 0.0f)
		{
			cerr << "Singular matrix in routine LUdecompose" << endl;
			return false;
			assert( FALSE );
		}

		vv[i] = 1.0f / big;
	}

	for ( j=0; j<n; j++ )
	{
		for ( i=0; i<j; i++ )
		{
			sum = a[i][j];
			for ( k=0; k<i; k++ )
				sum -= a[i][k] * a[k][j];
			a[i][j] = sum;
		}

		big = 0.0;
		for ( i=j; i<n; i++ )
		{
			sum = a[i][j];
			for ( k=0; k<j; k++ )
				sum -= a[i][k] * a[k][j];
			a[i][j] = sum;
			if ((dum = vv[i] * fabs(sum)) >= big)
			{
				big = dum;
				imax = i;
			}
		}

		if ( j!=imax )
		{
			for ( k=0; k<n; k++ )
			{
				dum = a[imax][k];
				a[imax][k] = a[j][k];
				a[j][k] = dum;
			}
			vv[imax] = vv[j];
		}

		index[j] = imax;
		if (a[j][j] == 0.0f) a[j][j] = EPS;

		if ( j!=n )
		{
			dum = 1.0f / a[j][j];
			for ( i=j+1; i<n; i++ )
				a[i][j] *= dum;
		}
	}
	return true;
}

void
math::matrixN::LUsubstitute( int* index, vectorN &b )
{
	assert( this->row() == this->column() );

	int n = this->row();
	matrixN &a = (*this);

	int i, ii = -1, ip, j;
	double sum;

	for ( i=0; i<n; i++ )
	{
		ip = index[i];
		sum = b[ip];
		b[ip] = b[i];

		if (ii>-1)
			for ( j=ii; j<i; j++ )
				sum -= a[i][j] * b[j];
		else
			if (sum)
				ii = i;

		b[i] = sum;
	}

	for ( i=n-1; i>=0; i-- )
	{
		sum = b[i];
		for ( j=i+1; j<n; j++ )
			sum -= a[i][j] * b[j];
		b[i] = sum / a[i][i];
	}
}

double
math::matrixN::LUinverse( matrixN &mat )
{
	assert( this->row() == this->column() );

	int n = this->row();

	static int* index;
	static int index_count = 0;
	if ( index_count<n )
	{
		if ( index_count>0 ) delete[] index;
		index_count = n;
		if ( index_count>0 ) index = new int[index_count];
	}

	int i, j;

	static vectorN b; b.setSize( n );
	mat.setSize( n, n );

	LUdecompose( index );

	double det=0;
	for( i=0; i<n; i++ )
		det += this->v[i][i];

	for( j=0; j<n; j++ )
	{
		for( i=0; i<n; i++ ) b[i] = 0;
		b[j] = 1.0;

		LUsubstitute( index, b );

		for( i=0; i<n; i++ )
			mat[i][j] = b[i];
	}

	return det;
}

matrixN&
math::matrixN::mergeUpDown( matrixN const& a, matrixN const& b )
{
	assert( a.column()==b.column() );
	matrixN &c = (*this);
	c.setSize( a.row()+b.row(), a.column() );

	int i, j;

	for( j=0; j<a.column(); j++ )
	{
		for( i=0; i<a.row(); i++ )
			c[i][j] = a[i][j];

		for( i=0; i<b.row(); i++ )
			c[i+a.row()][j] = b[i][j];
	}

	return c;
}

matrixN&
math::matrixN::mergeLeftRight( matrixN const& a, matrixN const& b )
{
	assert( a.row()==b.row() );
	matrixN &c = (*this);
	c.setSize( a.row(), a.column()+b.column() );

	int i, j;

	for( i=0; i<a.row(); i++ )
	{
		for( j=0; j<a.column(); j++ )
			c[i][j] = a[i][j];

		for( j=0; j<b.column(); j++ )
			c[i][j+a.column()] = b[i][j];
	}

	return c;
}

void
math::matrixN::splitUpDown( matrixN& a, matrixN& b )
{
	assert( this->row()%2 == 0 );
	matrixN &c = (*this);
	a.setSize( c.row()/2, c.column() );
	b.setSize( c.row()/2, c.column() );

	int i, j;

	for( j=0; j<a.column(); j++ )
	{
		for( i=0; i<a.row(); i++ )
			a[i][j] = c[i][j];

		for( i=0; i<b.row(); i++ )
			b[i][j] = c[i+a.row()][j];
	}
}

void
math::matrixN::splitLeftRight( matrixN& a, matrixN& b )
{
	assert( this->column()%2 == 0 );
	matrixN &c = (*this);
	a.setSize( c.row(), c.column()/2 );
	b.setSize( c.row(), c.column()/2 );

	int i, j;

	for( i=0; i<a.row(); i++ )
	{
		for( j=0; j<a.column(); j++ )
			a[i][j] = b[i][j];

		for( j=0; j<b.column(); j++ )
			b[i][j] = c[i][j+a.column()];
	}
}

void
math::matrixN::splitUpDown( matrixN& a, matrixN& b, int num )
{
	assert( this->row()>num );
	matrixN &c = (*this);
	a.setSize( num, c.column() );
	b.setSize( c.row()-num, c.column() );

	int i, j;

	for( j=0; j<a.column(); j++ )
	{
		for( i=0; i<a.row(); i++ )
			a[i][j] = c[i][j];

		for( i=0; i<b.row(); i++ )
			b[i][j] = c[i+a.row()][j];
	}
}

void
math::matrixN::splitLeftRight( matrixN& a, matrixN& b, int num )
{
	assert( this->column()>num );
	matrixN &c = (*this);
	a.setSize( c.row(), num );
	b.setSize( c.row(), c.column()-num );

	int i, j;

	for( i=0; i<a.row(); i++ )
	{
		for( j=0; j<a.column(); j++ )
			a[i][j] = b[i][j];

		for( j=0; j<b.column(); j++ )
			b[i][j] = c[i][j+a.column()];
	}
}

// used for SVdecompose
static double pythag( double a, double b )
{
	double pa = fabs( a );
	double pb = fabs( b );

	if ( pa > pb ) return pa * sqrt( 1.0f + SQR(pb / pa) );
	else return (pb==0.0f ? 0.0f : pb * sqrt(1.0f + SQR(pa / pb)));
}

void
math::matrixN::SVdecompose( vectorN& w, matrixN& v )
{
	matrixN &a = (*this);
	int m = a.row();
	int n = a.column();

	w.setSize( n );
	v.setSize( n, n );

	int flag, i, its, j, jj, k, l, nm;
	double anorm, c, f, g, h, s, scale, x, y, z;

  	static vectorN rv1; rv1.setSize( n );
	g = scale = anorm = 0.0;

	for( i=0; i<n; i++ )
	{
		l = i + 1;
		rv1[i] = scale * g;
		g = s = scale = 0.0;

		if ( i<m )
		{
			for ( k=i; k<m; k++ )
				scale += fabs(a[k][i]);

			if ( scale )
			{
				for ( k=i; k<m; k++ )
				{
					a[k][i] /= scale;
					s += a[k][i] * a[k][i];
				}

				f = a[i][i];
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				a[i][i] = f - g;

				for( j=l; j<n; j++ )
				{
					for ( s=0.0, k=i; k<m; k++ )
						s += a[k][i] * a[k][j];
					f = s / h;

					for ( k=i; k<m; k++ )
						a[k][j] += f * a[k][i];
				}

				for( k=i; k<m; k++ )
					a[k][i] *= scale;
			}
		}

		w[i] = scale * g;
		g = s = scale = 0.0;

		if ( i<m && i != n-1)
		{
			for( k=l; k<n; k++)
				scale += fabs(a[i][k]);

			if ( scale )
			{
				for( k=l; k<n; k++ )
				{
					a[i][k] /= scale;
					s += a[i][k] * a[i][k];
				}

				f = a[i][l];
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				a[i][l] = f - g;

				for ( k=l; k<n; k++ )
					rv1[k] = a[i][k] / h;

				for( j=l; j<m; j++ )
				{
					for( s=0.0, k=l; k<n; k++ )
						s += a[j][k] * a[i][k];

					for( k=l; k<n; k++ )
						a[j][k] += s * rv1[k];
				}

				for( k=l; k<n; k++ )
					a[i][k] *= scale;
			}
		}

		anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
	}

	for( i=n-1; i>=0; i-- )
	{
		if ( i<n-1 )
		{
			if ( g )
			{
				for( j=l; j<n; j++ )
					v[j][i] = (a[i][j] / a[i][l]) / g;

				for ( j=l; j<n; j++ )
				{
					for( s=0.0, k=l; k<n; k++ )
						s += a[i][k] * v[k][j];

					for( k=l; k<n; k++ )
						v[k][j] += s * v[k][i];
				}
			}

			for( j=l; j<n; j++ )
				v[i][j] = v[j][i] = 0.0;
		}

		v[i][i] = 1.0;
		g = rv1[i];
		l = i;
	}

	for( i=MIN(m, n)-1; i>=0; i-- )
	{
		l = i + 1;
		g = w[i];
		for( j=l; j<n; j++ )
		a[i][j] = 0.0;

		if ( g )
		{
			g = 1.0 / g;
			for( j=l; j<n; j++ )
			{
				for ( s=0.0, k=l; k<m; k++ )
					s += a[k][i] * a[k][j];

				f = (s / a[i][i]) * g;

				for( k=i; k<m; k++ )
					a[k][j] += f * a[k][i];
			}

			for( j=i; j<m; j++ )
				a[j][i] *= g;
		}
		else
			for( j=i; j<m; j++ )
				a[j][i] = 0.0;

		++a[i][i];
	}

	for( k=n-1; k>=0; k-- )
	{
		for( its=1; its<30; its++ )
		{
			flag = 1;
			for( l=k; l>=0; l-- )
			{
				nm = l - 1;
				if ((double) (fabs(rv1[l]) + anorm) == anorm)
				{
					flag = 0;
					break;
				}
				if ((double) (fabs(w[nm]) + anorm) == anorm) break;
			}

			if ( flag )
			{
				c = 0.0;
				s = 1.0;

				for( i=l; i<= k; i++ )
				{
					f = s * rv1[i];
					rv1[i] = c * rv1[i];

					if ((double) (fabs(f) + anorm) == anorm) break;

					g = w[i];
					h = pythag(f, g);
					w[i] = h;
					h = 1.0f / h;
					c = g * h;
					s = -f * h;

					for( j=0; j<m; j++ )
					{
						y = a[j][nm];
						z = a[j][i];
						a[j][nm] = y * c + z * s;
						a[j][i] = z * c - y * s;
					}
				}
			}

			z = w[k];
			if ( l == k )
			{
				if ( z < 0.0 )
				{
					w[k] = -z;
					for( j=0; j<n; j++ )
						v[j][k] = -v[j][k];
				}
				break;
			}

			if (its == 29)
				cerr << "no convergence in 30 svdcmp iterations" << endl;

			x = w[l];
			nm = k - 1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0f * h * y);
			g = pythag(f, 1.0f);
			f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
			c = s = 1.0f;

			for( j=l; j<=nm; j++ )
			{
				i = j + 1;
				g = rv1[i];
				y = w[i];
				h = s * g;
				g = c * g;
				z = pythag(f, h);
				rv1[j] = z;
				c = f / z;
				s = h / z;
				f = x * c + g * s;
				g = g * c - x * s;
				h = y * s;
				y *= c;

				for( jj=0; jj<n; jj++ )
				{
					x = v[jj][j];
					z = v[jj][i];
					v[jj][j] = x * c + z * s;
					v[jj][i] = z * c - x * s;
				}

				z = pythag(f, h);
				w[j] = z;

				if ( z )
				{
					z = 1.0f / z;
					c = f * z;
					s = h * z;
				}

				f = c * g + s * y;
				x = c * y - s * g;

				for( jj=0; jj<m; jj++ )
				{
					y = a[jj][j];
					z = a[jj][i];
					a[jj][j] = y * c + z * s;
					a[jj][i] = z * c - y * s;
				}
			}

			rv1[l] = 0.0;
			rv1[k] = f;
			w[k] = x;
		}
	}
}

void
math::matrixN::SVsubstitute( vectorN const& w, matrixN const& v,
            const vectorN& b, vectorN &x )
{
    assert( this->column() == w.getSize() );
    assert( this->column() == v.column() );
    assert( this->column() == v.row() );
    assert( this->row() == b.getSize() );
    assert( this->column() == x.getSize() );

    int m = this->row();
    int n = this->column();

    int jj,j,i;
    double s;
    static vectorN tmp; tmp.setSize(n);
    matrixN& u = *this;

    for (j=0;j<n;j++) {
        s=0.0;
        if (w[j]>EPS) {
            for (i=0;i<m;i++) s += u[i][j]*b[i];
            s /= w[j];
        }
        tmp[j]=s;
    }
    for (j=0;j<n;j++) {
        s=0.0;
        for (jj=0;jj<n;jj++) s += v[j][jj]*tmp[jj];
        x[j]=s;
    }
}

void
math::matrixN::SVinverse( matrixN &mat )
{
    int m = this->row();
    int n = this->column();

    static matrixN V; V.setSize( n, n );
    static vectorN w; w.setSize( n );

    static vectorN b; b.setSize( m );
    static vectorN x; x.setSize( n );

	mat.setSize( n, m );
    SVdecompose( w, V );

	int i, j;
    for( j=0; j<m; j++ )
    {
        for( i=0; i<m; i++ ) b[i] = 0;
        b[j] = 1.0;

        SVsubstitute( w, V, b, x );

        for( i=0; i<n; i++ )
            mat[i][j] = x[i];
    }
}

ostream& math::operator<<( ostream& os, matrixN const& a )
{
    for( int i=0; i< a.row(); i++ ) os << a.v[i] << endl;
    return os;
}

istream& math::operator>>( istream& is, matrixN& a )
{
    for( int i=0; i< a.row(); i++ ) is >> a.v[i];
    return is;
}

