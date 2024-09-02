#include "matrix.h"

Matrix::Matrix(int row, int col) : m_row(row), m_col(col)
{
    allocSpace();
}

Matrix::Matrix(const Matrix& matrix) : m_row(matrix.m_row), m_col(matrix.m_col)
{
    allocSpace();

    for (int i = 0; i < m_row; i++)
    {
        for (int j = 0; j < m_col; j++)
        {
            m_p[i][j] = matrix.m_p[i][j];
        }
    }
}

Matrix::~Matrix()
{
    for (int i = 0; i < m_row; i++)
    {
        delete[] m_p[i];
    }

    delete[] m_p;
}

void Matrix::InitializeToZero(void)
{
    for(int i = 0; i < m_row; i++)
    {
        for(int j = 0; j < m_col; j++)
        {
            m_p[i][j] = 0.0;
        }
    }
}

void Matrix::allocSpace(void)
{
    m_p = new double*[m_row];

    for (int i = 0; i < m_row; i++)
    {
        m_p[i] = new double[m_col];
    }
}

double Matrix::normVector(const Matrix& vector)
{
    double result = 0.0;

    for(auto i = 0; i < vector.m_row; i++)
    {
        result += (vector)(i,0) * (vector)(i, 0);
    }

    return std::sqrt(result);
}

Matrix& Matrix::operator=(const Matrix& matrix)
{
    if (this == &matrix)
    {
        return *this;
    }

    for(auto i = 0; i < m_row; i++)
    {
        for(auto j = 0; j < m_col; j++)
        {
            m_p[i][j] = matrix.m_p[i][j];
        }
    }

    return *this;
}

Matrix& Matrix::operator*=(double factor)
{
    for(auto i = 0; i < m_row; i++)
    {
        for(auto j = 0; j < m_col; j++)
        {
            m_p[i][j] *= factor;
        }
    }

    return *this;
}

Matrix& Matrix::operator*=(const Matrix& matrix)
{
    Matrix result(m_row, m_col);
    result.InitializeToZero();

    //k and j are swaped to make less access to the stack
    for (int i = 0; i < result.m_row; i++)
    {
        for (int k = 0; k < m_col; k++)
        {
            for (int j = 0; j < result.m_col; j++)
            {
                result.m_p[i][j] += (m_p[i][k] * matrix.m_p[k][j]);
            }
        }
    }

    return (*this = result);
}

Matrix operator*(const Matrix& matrix_1, const Matrix& matrix_2)
{
    Matrix result(matrix_1);

    return (result *= matrix_2);
}

Matrix operator*(double factor, const Matrix& matrix)
{
    Matrix result(matrix);

    return (result *= factor);
}

Matrix operator-(const Matrix& matrix)
{
    Matrix result(matrix);

    return (result = (-1) * matrix);
}

std::ostream& operator<<(std::ostream& ostream, const Matrix& matrix)
{
    for (int i = 0; i < matrix.m_row; i++)
    {
        ostream << matrix.m_p[i][0];

        for (int j = 1; j < matrix.m_col; j++)
        {
            ostream << " " << matrix.m_p[i][j];
        }

        ostream  << std::endl;
    }

    ostream << std::endl;

    return ostream;
}
