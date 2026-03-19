#ifndef _matrix_h_
#define _matrix_h_



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    MATRIX_SUCCESS = 0,
    MATRIX_ERROR = 1,
    MATRIX_CREATE_FAILED = 2,
    MATRIX_INVALID = 3,
    MATRIX_CANNOT_COMPUTE = 4,
    MATRIX_PIVOT_IS_ZERO = 5
} MatrixErrorCode_e;

typedef struct {
    double* pdata;
    unsigned int row;
    unsigned int col;
} Matrix_t;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

MatrixErrorCode_e matrix(Matrix_t* const m, unsigned int row, unsigned int col, double* data);
MatrixErrorCode_e matrix_identity(Matrix_t* const m, unsigned int size, double* data);
MatrixErrorCode_e matrix_get(const Matrix_t* const m, unsigned int r, unsigned int c, double* value);
MatrixErrorCode_e matrix_set(Matrix_t* const m, unsigned int r, unsigned int c, double value);
MatrixErrorCode_e matrix_copy(const Matrix_t* const m, Matrix_t* const out);

MatrixErrorCode_e matrix_add(const Matrix_t* const A, const Matrix_t* const B, Matrix_t* const out);
MatrixErrorCode_e matrix_sub(const Matrix_t* const A, const Matrix_t* const B, Matrix_t* const out);
MatrixErrorCode_e matrix_scalar_mul(const Matrix_t* const m, double scalar, Matrix_t* const out);
MatrixErrorCode_e matrix_mul(const Matrix_t* const A, const Matrix_t* const B, Matrix_t* const out);
MatrixErrorCode_e matrix_transpose(const Matrix_t* const m, Matrix_t* const out);
MatrixErrorCode_e matrix_to_upper_triangular(const Matrix_t* const m, Matrix_t* const out);
MatrixErrorCode_e matrix_to_lower_triangular(const Matrix_t* const m, Matrix_t* const out);
MatrixErrorCode_e matrix_determinant(const Matrix_t* const m, double* out);
MatrixErrorCode_e matrix_inverse(const Matrix_t* const m, Matrix_t* const out);

#endif
