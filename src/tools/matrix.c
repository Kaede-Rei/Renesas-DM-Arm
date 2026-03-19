#include "matrix.h"

#include <math.h>
#include <sys/reent.h>

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //


MatrixErrorCode_e matrix(Matrix_t* const m, unsigned int row, unsigned int col, double* data) {
    if(m == NULL || data == NULL || row == 0 || col == 0) return MATRIX_CREATE_FAILED;

    m->pdata = data;
    m->row = row;
    m->col = col;

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_identity(Matrix_t* const m, unsigned int size, double* data) {
    if(m == NULL || data == NULL || size == 0) return MATRIX_CREATE_FAILED;

    m->pdata = data;
    m->row = size;
    m->col = size;

    for(unsigned int i = 0; i < size; ++i) {
        for(unsigned int j = 0; j < size; ++j) {
            m->pdata[i * size + j] = (i == j) ? 1.0 : 0.0;
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_get(const Matrix_t* const m, unsigned int r, unsigned int c, double* value) {
    if(m == NULL || m->pdata == NULL || r >= m->row || c >= m->col) return MATRIX_INVALID;
    if(value == NULL) return MATRIX_ERROR;

    *value = m->pdata[r * m->col + c];

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_set(Matrix_t* const m, unsigned int r, unsigned int c, double value) {
    if(m == NULL || m->pdata == NULL || r >= m->row || c >= m->col) return MATRIX_INVALID;

    m->pdata[r * m->col + c] = value;

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_copy(const Matrix_t* const m, Matrix_t* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < m->row; ++i) {
        for(unsigned int j = 0; j < m->col; ++j) {
            out->pdata[i * m->col + j] = m->pdata[i * m->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_add(const Matrix_t* const A, const Matrix_t* const B, Matrix_t* const out) {
    if(A == NULL || A->pdata == NULL || B == NULL || B->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(A->row != B->row || A->col != B->col || A->row != out->row || A->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < A->row; ++i) {
        for(unsigned int j = 0; j < A->col; ++j) {
            out->pdata[i * A->col + j] = A->pdata[i * A->col + j] + B->pdata[i * B->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_sub(const Matrix_t* const A, const Matrix_t* const B, Matrix_t* const out) {
    if(A == NULL || A->pdata == NULL || B == NULL || B->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(A->row != B->row || A->col != B->col || A->row != out->row || A->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < A->row; ++i) {
        for(unsigned int j = 0; j < A->col; ++j) {
            out->pdata[i * A->col + j] = A->pdata[i * A->col + j] - B->pdata[i * B->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_scalar_mul(const Matrix_t* const m, double scalar, Matrix_t* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < m->row; ++i) {
        for(unsigned int j = 0; j < m->col; ++j) {
            out->pdata[i * m->col + j] = m->pdata[i * m->col + j] * scalar;
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_mul(const Matrix_t* const A, const Matrix_t* const B, Matrix_t* const out) {
    if(A == NULL || A->pdata == NULL || B == NULL || B->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(A->col != B->row || A->row != out->row || B->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < A->row; ++i) {
        for(unsigned int j = 0; j < B->col; ++j) {
            double sum = 0.0;
            for(unsigned int k = 0; k < A->col; ++k) {
                sum += A->pdata[i * A->col + k] * B->pdata[k * B->col + j];
            }
            out->pdata[i * out->col + j] = sum;
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_transpose(const Matrix_t* const m, Matrix_t* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != out->col || m->col != out->row) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < m->row; ++i) {
        for(unsigned int j = 0; j < m->col; ++j) {
            out->pdata[j * out->col + i] = m->pdata[i * m->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_to_upper_triangular(const Matrix_t* const m, Matrix_t* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != m->col || m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;

    matrix_copy(m, out);

    // TODO: 部分主元选择未实现

    for(unsigned int i = 0; i < out->row; ++i) {
        double pivot = out->pdata[i * out->col + i];
        if(fabs(pivot) <= 1e-10) return MATRIX_PIVOT_IS_ZERO;

        for(unsigned int j = i + 1; j < out->row; ++j) {
            double factor = out->pdata[j * out->col + i] / pivot;

            for(unsigned int k = i; k < out->col; ++k) {
                out->pdata[j * out->col + k] -= factor * out->pdata[i * out->col + k];
            }
            out->pdata[j * out->col + i] = 0.0;
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_to_lower_triangular(const Matrix_t* const m, Matrix_t* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != m->col || m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;

    matrix_copy(m, out);

    // TODO: 部分主元选择未实现

    for(unsigned int i = 0; i < out->row; ++i) {
        double pivot = out->pdata[i * out->col + i];
        if(fabs(pivot) <= 1e-10) return MATRIX_PIVOT_IS_ZERO;

        for(unsigned int j = 0; j < i; ++j) {
            double factor = out->pdata[j * out->col + i] / pivot;

            for(unsigned int k = 0; k <= i; ++k) {
                out->pdata[j * out->col + k] -= factor * out->pdata[i * out->col + k];
            }
            out->pdata[j * out->col + i] = 0.0;
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_determinant(const Matrix_t* const m, double* out) {
    if(m == NULL || m->pdata == NULL || out == NULL) return MATRIX_INVALID;
    if(m->row != m->col) return MATRIX_CANNOT_COMPUTE;

    Matrix_t temp;
    temp.col = m->col;
    temp.row = m->row;
    double temp_data[m->row * m->col];
    temp.pdata = temp_data;

    MatrixErrorCode_e error = matrix_to_upper_triangular(m, &temp);
    if(error != MATRIX_SUCCESS) {
        return error;
    }

    *out = 1.0;
    for(unsigned int i = 0; i < temp.row; ++i) {
        *out *= temp.pdata[i * temp.col + i];
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode_e matrix_inverse(const Matrix_t* const m, Matrix_t* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != m->col || m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;

    double det;
    if(matrix_determinant(m, &det) != MATRIX_SUCCESS || det == 0.0) return MATRIX_CANNOT_COMPUTE;

    Matrix_t identity;
    identity.row = m->row;
    identity.col = m->col;
    double identity_data[m->row * m->col];
    identity.pdata = identity_data;
    matrix_identity(&identity, m->row, identity_data);

    // TODO: 高斯-若尔当消元法未实现

    return MATRIX_ERROR;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //


