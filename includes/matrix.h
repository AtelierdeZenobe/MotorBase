/**
 * @brief Based on the matrix project in GitHub : https://github.com/akalicki/matrix
 * This version only contains what is needed by the robot to move on.
 * It also includes some modifications from the original release and improvements.
 */

 #ifndef __MATRIX_H__
 #define __MATRIX_H__

 #include <cmath>
 #include <iostream>

 class Matrix
 {
     
     public:
        Matrix() = delete;
        Matrix(int row, int col);
        Matrix(const Matrix& matrix);
        ~Matrix();

        void InitializeToZero(void);
    
        const int m_row;
        const int m_col;

        inline double& operator()(int row, int col) const
        {
            return m_p[row][col];
        }

        Matrix& operator=(const Matrix& matrix);
        Matrix& operator*=(double factor);
        Matrix& operator*=(const Matrix& matrix);

        friend std::ostream& operator<<(std::ostream& ostream, const Matrix& matrix);

        static double normVector(const Matrix& vector);

     private:
        double** m_p;

        void allocSpace(void);
    
 };

Matrix operator*(double factor, const Matrix& matrix);
Matrix operator*(const Matrix& matrix_1, const Matrix& matrix_2);
Matrix operator-(const Matrix& matrix);

#endif