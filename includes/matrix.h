#pragma once

#include <cmath>
#include <iostream>

/**
  * @brief Class used to create matrixes
  *
  * @note Based on the matrix project in GitHub : https://github.com/akalicki/matrix
  */
 class Matrix
 {     
     public:
        const int m_row;
        const int m_col;

        Matrix() = delete;
        ~Matrix();
        
        /**
          * @brief Constructor to use when number of rows and cols is known
          *
          * @param row : number of rows
          * @param col : number of columns
          */
        Matrix(int row, int col);

        /**
          * @brief Constructor to use when you need a copy of an already instanciated matrix
          *
          * @param matrix : the already instanciated matrix
          */
        Matrix(const Matrix& matrix);
        
        /**
          * @brief Function used to calculate the norm of a matrix used as a vector
          * @attention Number of columns of the matrix must be equal to 1
          *
          * @param vector : Matrix used as a vector
          * @param normVector : Variable which will contain the calculated value of the vector
          */
        static bool normVector(const Matrix& vector, double* normVector);

        /**
          * @brief Overleaded operator() used to make acces to any element of the matrix easier
          *
          * @param row : number of rows
          * @param col : number of columns
          */
        inline double& operator()(int row, int col) const
        {
            return m_p[row][col];
        }

        /**
          * @brief Overloaded operator= used to create a copy of a matrix
          *
          * @param matrix : matrix to be copied
          */
        Matrix& operator=(const Matrix& matrix);

        /**
          * @brief Overloaded operator*= used to multiplicate a matrix by a double value
          *
          * @param factor : double value used to multiplicate the matrix
          */
        Matrix& operator*=(const double factor);

        /**
          * @brief Overloaded operator*= used to multiply a matrix by a matrix
          *
          * @param factor : matrix used to multiply the matrix
          */
        Matrix& operator*=(const Matrix& matrix);

        /**
          * @brief Overloaded operator<< used to show the matrix
          *
          * @param ostream : stream where the matrix is showed
          * @param matrix : matrix to be showed
          */
        friend std::ostream& operator<<(std::ostream& ostream, const Matrix& matrix);

     private:
        // Pointer which contains all the elements of the matrix
        double** m_p;

        // Variable used when the matrix has to be a vector to verify the number of columns is really equal to 1
        static const int COL_VECTOR = 1;

        /**
          * @brief Function used to alloc required space when a matrix is created
          */
        void allocSpace(void);

        /**
          * @brief Function used to initialize all elements of the matrix to 0
          */
        void InitializeToZero(void);
 };

/**
  * @brief Overloaded operator* used to multiply a matrix by a double value
  * @note Returns the result in a new matrix
  *
  * @param factor : the double value to be multiplied
  * @param matrix : the matrix to be multiplied
  */
Matrix operator*(const double factor, const Matrix& matrix);

/**
  * @brief Overloaded operator* used to make a matrix product
  * @note Returns the result in a new matrix
  * @attention Follows the matrix product order : [result] = [matrix_1] x [matrix_2]
  *
  * @param matrix_1 : the left matrix to be multiplied
  * @param matrix_2 : the right matrix to be multiplied
  */
Matrix operator*(const Matrix& matrix_1, const Matrix& matrix_2);

/**
  * @brief Overloaded operator- used to change the sign of the matrix
  * @note Returns the result in a new matrix
  *
  * @param matrix : the matrix to be sign-changed
  */
Matrix operator-(const Matrix& matrix);