#include "matrix.h"

#include <assert.h>
#include <math.h>

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 创建一个矩阵结构体
 * @param m 输出的矩阵结构体
 * @param row 矩阵的行数
 * @param col 矩阵的列数
 * @param data 矩阵数据的指针，必须至少包含 row * col 个 float 元素
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix(Matrix* const m, unsigned int row, unsigned int col, float* data) {
    if(m == NULL || data == NULL || row == 0 || col == 0) return MATRIX_CREATE_FAILED;

    m->pdata = data;
    m->row = row;
    m->col = col;

    return MATRIX_SUCCESS;
}

/**
 * @brief 创建一个单位矩阵
 * @param m 输出的矩阵结构体
 * @param size 矩阵的行列数，必须为正整数
 * @param data 矩阵数据的指针，必须至少包含 size * size 个 float 元素
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_identity(Matrix* const m, unsigned int size, float* data) {
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

/**
 * @brief 获取矩阵中指定位置的元素值
 * @param m 输入的矩阵结构体
 * @param r 行索引，范围 0 到 m->row - 1
 * @param c 列索引，范围 0 到 m->col - 1
 * @param value 输出的元素值指针
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_get(const Matrix* const m, unsigned int r, unsigned int c, float* value) {
    if(m == NULL || m->pdata == NULL || r >= m->row || c >= m->col) return MATRIX_INVALID;
    if(value == NULL) return MATRIX_ERROR;

    *value = m->pdata[r * m->col + c];

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_set(Matrix* const m, unsigned int r, unsigned int c, float value) {
    if(m == NULL || m->pdata == NULL || r >= m->row || c >= m->col) return MATRIX_INVALID;

    m->pdata[r * m->col + c] = value;

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_copy(const Matrix* const m, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < m->row; ++i) {
        for(unsigned int j = 0; j < m->col; ++j) {
            out->pdata[i * m->col + j] = m->pdata[i * m->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_add(const Matrix* const A, const Matrix* const B, Matrix* const out) {
    if(A == NULL || A->pdata == NULL || B == NULL || B->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(A->row != B->row || A->col != B->col || A->row != out->row || A->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < A->row; ++i) {
        for(unsigned int j = 0; j < A->col; ++j) {
            out->pdata[i * A->col + j] = A->pdata[i * A->col + j] + B->pdata[i * B->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_sub(const Matrix* const A, const Matrix* const B, Matrix* const out) {
    if(A == NULL || A->pdata == NULL || B == NULL || B->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(A->row != B->row || A->col != B->col || A->row != out->row || A->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < A->row; ++i) {
        for(unsigned int j = 0; j < A->col; ++j) {
            out->pdata[i * A->col + j] = A->pdata[i * A->col + j] - B->pdata[i * B->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_scalar_mul(const Matrix* const m, float scalar, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < m->row; ++i) {
        for(unsigned int j = 0; j < m->col; ++j) {
            out->pdata[i * m->col + j] = m->pdata[i * m->col + j] * scalar;
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_mul(const Matrix* const A, const Matrix* const B, Matrix* const out) {
    if(A == NULL || A->pdata == NULL || B == NULL || B->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(A->col != B->row || A->row != out->row || B->col != out->col) return MATRIX_CANNOT_COMPUTE;
    if(A == out || B == out) return MATRIX_ERROR;

    for(unsigned int i = 0; i < A->row; ++i) {
        for(unsigned int j = 0; j < B->col; ++j) {
            float sum = 0.0;
            for(unsigned int k = 0; k < A->col; ++k) {
                sum += A->pdata[i * A->col + k] * B->pdata[k * B->col + j];
            }
            out->pdata[i * out->col + j] = sum;
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_transpose(const Matrix* const m, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != out->col || m->col != out->row) return MATRIX_CANNOT_COMPUTE;
    if(m == out) return MATRIX_INPLACE;

    for(unsigned int i = 0; i < m->row; ++i) {
        for(unsigned int j = 0; j < m->col; ++j) {
            out->pdata[j * out->col + i] = m->pdata[i * m->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_to_upper_triangular(const Matrix* const m, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != m->col || m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;
    if(m == out) return MATRIX_INPLACE;

    unsigned int n = m->row;
    matrix_copy(m, out);

    for(unsigned int i = 0; i < n; ++i) {
        unsigned int max_row = i;
        float max_val = fabsf(out->pdata[i * n + i]);

        for(unsigned int k = i + 1; k < n; ++k) {
            float val = fabsf(out->pdata[k * n + i]);
            if(val > max_val) {
                max_val = val;
                max_row = k;
            }
        }

        if(max_val < 1e-8f) return MATRIX_PIVOT_IS_ZERO;
        if(max_row != i) {
            for(unsigned int j = 0; j < n; ++j) {
                float tmp = out->pdata[i * n + j];
                out->pdata[i * n + j] = out->pdata[max_row * n + j];
                out->pdata[max_row * n + j] = tmp;
            }
        }

        float pivot = out->pdata[i * n + i];
        for(unsigned int j = i + 1; j < n; ++j) {
            float factor = out->pdata[j * n + i] / pivot;

            for(unsigned int k = i; k < n; ++k) {
                out->pdata[j * n + k] -= factor * out->pdata[i * n + k];
            }

            out->pdata[j * n + i] = 0.0f;
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_to_lower_triangular(const Matrix* const m, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != m->col || m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;
    if(m == out) return MATRIX_INPLACE;

    unsigned int n = m->row;
    matrix_copy(m, out);

    for(int i = (int)n - 1; i >= 0; --i) {
        unsigned int max_row = (unsigned int)i;
        float max_val = fabsf(out->pdata[(unsigned int)i * n + (unsigned int)i]);

        for(int k = i - 1; k >= 0; --k) {
            float val = fabsf(out->pdata[(unsigned int)k * n + (unsigned int)i]);
            if(val > max_val) {
                max_val = val;
                max_row = (unsigned int)k;
            }
        }

        if(max_val < 1e-8f) return MATRIX_PIVOT_IS_ZERO;
        if(max_row != (unsigned int)i) {
            for(unsigned int j = 0; j < n; ++j) {
                float tmp = out->pdata[(unsigned int)i * n + j];
                out->pdata[(unsigned int)i * n + j] = out->pdata[max_row * n + j];
                out->pdata[max_row * n + j] = tmp;
            }
        }

        float pivot = out->pdata[(unsigned int)i * n + (unsigned int)i];
        for(int j = i - 1; j >= 0; --j) {
            float factor = out->pdata[(unsigned int)j * n + (unsigned int)i] / pivot;

            for(int k = i; k >= 0; --k) {
                out->pdata[(unsigned int)j * n + (unsigned int)k] -= factor * out->pdata[(unsigned int)i * n + (unsigned int)k];
            }

            out->pdata[(unsigned int)j * n + (unsigned int)i] = 0.0f;
        }
    }

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_determinant(const Matrix* const m, float* out) {
    if(m == NULL || m->pdata == NULL || out == NULL)
        return MATRIX_INVALID;

    if(m->row != m->col)
        return MATRIX_CANNOT_COMPUTE;

    unsigned int n = m->row;

    float temp_data[n * n];
    Matrix temp;
    matrix(&temp, n, n, temp_data);
    matrix_copy(m, &temp);
    int swap_count = 0;

    for(unsigned int i = 0; i < n; ++i) {
        unsigned int max_row = i;
        float max_val = fabsf(temp.pdata[i * n + i]);

        for(unsigned int k = i + 1; k < n; ++k) {
            float val = fabsf(temp.pdata[k * n + i]);
            if(val > max_val) {
                max_val = val;
                max_row = k;
            }
        }

        if(max_val < 1e-8f) {
            *out = 0.0f;
            return MATRIX_SUCCESS;
        }

        if(max_row != i) {
            for(unsigned int j = 0; j < n; ++j) {
                float tmp = temp.pdata[i * n + j];
                temp.pdata[i * n + j] = temp.pdata[max_row * n + j];
                temp.pdata[max_row * n + j] = tmp;
            }
            swap_count++;
        }

        float pivot = temp.pdata[i * n + i];
        for(unsigned int j = i + 1; j < n; ++j) {
            float factor = temp.pdata[j * n + i] / pivot;

            for(unsigned int k = i; k < n; ++k) {
                temp.pdata[j * n + k] -= factor * temp.pdata[i * n + k];
            }
        }
    }

    float det = 1.0f;
    for(unsigned int i = 0; i < n; ++i) {
        det *= temp.pdata[i * n + i];
    }

    if(swap_count % 2 != 0) {
        det = -det;
    }
    *out = det;

    return MATRIX_SUCCESS;
}

MatrixErrorCode matrix_inverse(const Matrix* const m, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != m->col || m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;
    if(m == out) return MATRIX_INPLACE;

    unsigned int n = m->row;
    matrix_create(aug, n, 2 * n);

    for(unsigned int i = 0; i < n; i++) {
        for(unsigned int j = 0; j < n; j++) {
            aug.pdata[i * aug.col + j] = m->pdata[i * m->col + j];
            aug.pdata[i * aug.col + (j + n)] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for(unsigned int i = 0; i < n; i++) {

        unsigned int max_row = i;
        float max_val = fabsf(aug.pdata[i * aug.col + i]);

        for(unsigned int k = i + 1; k < n; k++) {
            float val = fabsf(aug.pdata[k * aug.col + i]);
            if(val > max_val) {
                max_val = val;
                max_row = k;
            }
        }

        if(max_val < 1e-8f)
            return MATRIX_PIVOT_IS_ZERO;

        if(max_row != i) {
            for(unsigned int j = 0; j < 2 * n; j++) {
                float tmp = aug.pdata[i * aug.col + j];
                aug.pdata[i * aug.col + j] = aug.pdata[max_row * aug.col + j];
                aug.pdata[max_row * aug.col + j] = tmp;
            }
        }

        float pivot = aug.pdata[i * aug.col + i];
        for(unsigned int j = 0; j < 2 * n; j++) {
            aug.pdata[i * aug.col + j] /= pivot;
        }

        for(unsigned int k = 0; k < n; k++) {
            if(k == i) continue;

            float factor = aug.pdata[k * aug.col + i];

            for(unsigned int j = 0; j < 2 * n; j++) {
                aug.pdata[k * aug.col + j] -= factor * aug.pdata[i * aug.col + j];
            }
        }
    }

    for(unsigned int i = 0; i < n; i++) {
        for(unsigned int j = 0; j < n; j++) {
            out->pdata[i * out->col + j] = aug.pdata[i * aug.col + (j + n)];
        }
    }

    return MATRIX_SUCCESS;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //


