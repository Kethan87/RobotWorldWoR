#include <cassert>
#include <stdexcept>
#include <numeric>
#include <cmath>
#include <utility>
#include <iomanip>

/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( T value)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) = value;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const std::initializer_list< T >& aList)
{
	// Check the arguments
	assert( aList.size() == M * N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column, ++row_iter)
		{
			matrix.at( row).at( column) = *row_iter;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const std::initializer_list< std::initializer_list< T > >& aList)
{
	// Check the arguments, the static assert assures that there is at least 1 M and 1 N!
	assert( aList.size() == M && (*aList.begin()).size() == N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < aList.size(); ++row, ++row_iter)
	{
		auto column_iter = (*row_iter).begin();
		for (std::size_t column = 0; column < (*row_iter).size(); ++column, ++column_iter)
		{
			matrix.at( row).at( column) = *column_iter;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const Matrix< T, M, N >& aMatrix) :
				matrix( aMatrix.matrix)
{
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::array< T, N >& Matrix< T, M, N >::at( std::size_t aRowIndex)
{
	return matrix.at( aRowIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const std::array< T, N >& Matrix< T, M, N >::at( std::size_t aRowIndex) const
{
	return matrix.at( aRowIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
T& Matrix< T, M, N >::at( 	std::size_t aRowIndex,
							std::size_t aColumnIndex)
{
	return matrix.at( aRowIndex).at( aColumnIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const T& Matrix< T, M, N >::at( std::size_t aRowIndex,
								std::size_t aColumnIndex) const
{
	return matrix.at( aRowIndex).at( aColumnIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::array< T, N >& Matrix< T, M, N >::operator[]( std::size_t aRowIndex)
{
	return matrix[aRowIndex];
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const std::array< T, N >& Matrix< T, M, N >::operator[]( std::size_t aRowIndex) const
{
	return matrix[aRowIndex];
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator=( const Matrix< T, M, N >& rhs)
{
	if (this != &rhs)
	{
		matrix = rhs.matrix;
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
bool Matrix< T, M, N >::operator==( const Matrix< T, M, N >& rhs) const
{
	return matrix == rhs.matrix;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N >& Matrix< T, M, N >::operator*=( const T2& scalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) *= scalar;
		}
	}
	return *this;
}
/*
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N > Matrix< T, M, N >::operator*( const T2& scalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix< T, M, N > result( *this);
	return result *= scalar;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N >& Matrix< T, M, N >::operator/=( const T2& aScalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) /= aScalar;
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N > Matrix< T, M, N >::operator/( const T2& aScalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix< T, M, N > result( *this);
	return result /= aScalar;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator+=( const Matrix< T, M, N >& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] += rhs.at( row, column);
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::operator+( const Matrix< T, M, N >& rhs) const
{
	Matrix< T, M, N > result( *this);
	return result += rhs;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator-=( const Matrix< T, M, N >& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] -= rhs.at( row, column);
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::operator-( const Matrix< T, M, N >& rhs) const
{
	Matrix< T, M, N > result( *this);
	return result -= rhs;
}
///**
// * (M, N) * (N, P) -> (M, P)
// */


template< typename T, std::size_t M, std::size_t N >
template< std::size_t columns >
Matrix< T, M, columns > Matrix< T, M, N >::operator*( const Matrix< T, N, columns >& rhs) const
{
    Matrix< T, M, columns > result;

    for(std::size_t column = 0; column < columns; ++column)
    {
    	for(std::size_t row = 0; row < M; ++row)
    	{
    		T temp = 0;
			for (std::size_t cell = 0; cell < N; ++cell)
			{
				temp += matrix[row][cell] * rhs.at(cell, column);
			}
			result.at(row, column) = temp;
    	}
    }

    return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, N, M > Matrix< T, M, N >::transpose() const
{
	Matrix< T, N, M > result = *this;
	for(std::size_t row = 0; row < N; ++row)
	{
		for(std::size_t column = 0; column < M; ++column)
		{
			result.at(column, row) = matrix[row][column];
		}
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::identity() const
{
	// TODO Implement this function
	Matrix< T, N, M > result = *this;
	for(std::size_t row = 0; row < M; ++row)
	{
		for(std::size_t column = 0; column < N; ++column)
		{
			if(row == column)
			{
				result.at(row, column) = 1;
			} else {
				result.at(row, column) = 0;
			}
		}
	}
	return result;
}
/**
 *
 */

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::gauss() const
{
	// TODO Implement this function
	Matrix< T, M, N> result = *this;
	for(std::size_t column = 0; column < N - 1; ++column)
	{
		std::size_t pivotRowIndex = column;
		for(std::size_t row = column + 1; row < M; ++row)
		{
			if (abs(result.at(row, column)) > abs(result.at(pivotRowIndex, column)))
			{
				pivotRowIndex = row;
			}
		}
		if(pivotRowIndex != column)
		{
			for (std::size_t cell = 0; cell < N; ++cell)
			{
				std::swap(result.at(column, cell),result.at(pivotRowIndex, cell));
			}
		}
		std::size_t pivotCell = result.at(column, column);

		for(std::size_t columnCell = 0; columnCell < N; ++columnCell)
		{
			result.at(column, columnCell) = result.at(column, columnCell) / pivotCell;
		}

		for(std::size_t rowCell = column + 1; rowCell < M; ++rowCell)
		{
			if(result.at(rowCell, column) != 0)
			{
				T pivotMultiplicator = result.at(rowCell, column);
				for(std::size_t columnCell = 0; columnCell < N; ++columnCell)
				{
					result.at(rowCell, columnCell) = result.at(rowCell, columnCell) - (pivotMultiplicator * result.at(column,columnCell));
				}
			}
		}
	}
	return result;
}

//This function performs the Gaussian elimination algorithm to transform the input matrix into its row echelon form and returns the result as a new matrix. Note that this implementation assumes that the input matrix is not singular, as Gaussian elimination may not work correctly for singular matrices.

/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::gaussJordan() const
{
	// TODO Implement this function
	Matrix< T, M, N> result = *this;
	result = result.gauss();

	for(int column = 2; column > -1; --column)
	{
		for(int row = column - 1; row > -1; --row)
		{
			if(result.at(row, column) != 0)
			{
				T pivotMultiplicator = result.at(row, column);
				for(int columnCell = 3; columnCell > -1; --columnCell)
				{
					result.at(row, columnCell) = result.at(row, columnCell) - (pivotMultiplicator * result.at(column,columnCell));
				}
			}
//			std::cout << "-----------------------" << std::endl;
		}
	}
	return result;
}
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, 1 > Matrix< T, M, N >::solve() const
{
	// TODO Implement this function
	Matrix < T, M, 1 > result;
	Matrix < T, M, N> appliedGausJordanMatrix = *this;
	appliedGausJordanMatrix = appliedGausJordanMatrix.gaussJordan();
	for(int cell = 0; cell < static_cast<int>(M); ++cell)
	{
		result.at(cell,0) = appliedGausJordanMatrix.at(cell,3);
	}

	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::inverse() const
{
    Matrix< T, M, N + N > working_matrix;
    Matrix< T, M, N > identity_matrix = identity();
    Matrix< T, M, N > result;

    for(std::size_t row = 0; row < M; ++row)
    {
        for(std::size_t column = 0; column < N; ++column)
        {
            working_matrix.at(row, column) = matrix[row][column];
            working_matrix.at(row, column + N) = identity_matrix.at(row, column);
        }
    }

    for (std::size_t iterator = 0; iterator < M; ++iterator)
    {
        std::size_t pivot_row_index = iterator;

        for (std::size_t row = iterator + 1; row < M; ++row)
        {
            if (abs(working_matrix.at(row, iterator)) > abs(working_matrix.at(pivot_row_index, iterator)))
            {
                pivot_row_index = row;
            }
        }

        if (pivot_row_index != iterator)
        {
            for (std::size_t col = 0; col < N + N; ++col)
            {
                std::swap(working_matrix.at(iterator, col), working_matrix.at(pivot_row_index, col));
            }
        }

        T pivot_value = working_matrix.at(iterator, iterator);
        for (std::size_t col = iterator; col < N + N; ++col)
        {
            working_matrix.at(iterator, col) /= pivot_value;
        }

        for (std::size_t row = 0; row < M; ++row)
        {
            if (row != iterator)
            {
                T factor = working_matrix.at(row, iterator);

                for (std::size_t col = iterator; col < N + N; ++col)
                {
                    working_matrix.at(row, col) -= factor * working_matrix.at(iterator, col);
                }
            }
        }
    }


    for(std::size_t row = 0; row < M; ++row)
    {
        for(std::size_t column = 0; column < N; ++column)
        {
            result.at(row, column) = working_matrix.at(row, column + N);
        }
    }

    return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::string Matrix< T, M, N >::to_string() const
{
	std::string result = "Matrix<" + std::to_string( N) + "," + std::to_string( M) + ">\n{\n";
	for (std::size_t i = 0; i < M; ++i)
	{
		for (std::size_t j = 0; j < N; ++j)
		{
			result += std::to_string( matrix[i][j]) + ",";
		}
		result += "\n";
	}
	result += "}";
	return result;
}
/**
 *
 */
template< typename T, const std::size_t N >
bool equals(	const Matrix< T, 1, N >& lhs,
				const Matrix< T, 1, N >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	T actualPrecision = aPrecision * aFactor;
	for (std::size_t column = 0; column < N; ++column)
	{
		if(std::abs(lhs[0][column] - rhs[0][column]) > actualPrecision)
		{
			return false;
		}
	}
	return true;
}
/**
 *
 */
template< typename T, const std::size_t M >
bool equals(	const Matrix< T, M, 1 >& lhs,
				const Matrix< T, M, 1 >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	T actualPrecision = aPrecision * aFactor;
	for (std::size_t row = 0; row < M; ++row)
	{
		if(std::abs(lhs[row][0] - rhs[row][0]) > actualPrecision)
		{
			return false;
		}
	}
	return true;
}
/**
 *
 */
template< typename T, const std::size_t M, const std::size_t N >
bool equals(	const Matrix< T, M, N >& lhs,
				const Matrix< T, M, N >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	T actualPrecision = aPrecision * aFactor;
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			if(std::abs(lhs[row][column] - rhs[row][column]) > actualPrecision)
			{
				return false;
			}
		}
	}
	return true;
}



