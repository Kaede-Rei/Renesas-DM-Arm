#ifndef _matrix_h_
#define _matrix_h_



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    MATRIX_SUCCESS = 0,
    MATRIX_ERROR,
    MATRIX_CREATE_FAILED,
    MATRIX_INVALID,
    MATRIX_CANNOT_COMPUTE,
    MATRIX_PIVOT_IS_ZERO,
    MATRIX_INPLACE,
} MatrixErrorCode;

typedef struct {
    float* pdata;
    unsigned int row;
    unsigned int col;
} Matrix;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

#define matrix_create(name, row, col) \
    float name##_data[row * col]; \
    Matrix name; \
    matrix(&name, row, col, name##_data)
#define matrix_identity_create(name, size) \
    float name##_data[size * size]; \
    Matrix name; \
    matrix_identity(&name, size, name##_data)

MatrixErrorCode matrix(Matrix* const m, unsigned int row, unsigned int col, float* data);
MatrixErrorCode matrix_identity(Matrix* const m, unsigned int size, float* data);
MatrixErrorCode matrix_get(const Matrix* const m, unsigned int r, unsigned int c, float* value);
MatrixErrorCode matrix_set(Matrix* const m, unsigned int r, unsigned int c, float value);
MatrixErrorCode matrix_copy(const Matrix* const m, Matrix* const out);

MatrixErrorCode matrix_add(const Matrix* const A, const Matrix* const B, Matrix* const out);
MatrixErrorCode matrix_sub(const Matrix* const A, const Matrix* const B, Matrix* const out);
MatrixErrorCode matrix_scalar_mul(const Matrix* const m, float scalar, Matrix* const out);
MatrixErrorCode matrix_mul(const Matrix* const A, const Matrix* const B, Matrix* const out);
MatrixErrorCode matrix_transpose(const Matrix* const m, Matrix* const out);
MatrixErrorCode matrix_to_upper_triangular(const Matrix* const m, Matrix* const out);
MatrixErrorCode matrix_to_lower_triangular(const Matrix* const m, Matrix* const out);
MatrixErrorCode matrix_determinant(const Matrix* const m, float* out);
MatrixErrorCode matrix_inverse(const Matrix* const m, Matrix* const out);

#endif
