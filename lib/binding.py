"""Python to CPP/Eigen bindings

This module has some helper functions and tools to help manage the mapping
of numpy arrays to eigen array

Notes
-----
    This is an example of an indented section. It's like any other section,
    but the body is indented to help it stand out from surrounding text.
    
    Numpy and Eigen store data in different formats. However, it is relatively 
    easy to convert between them.

Attributes
----------

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""

import numpy as np

def numpy_to_eigen(A, dtype=np.float64):
    r"""Convert from the default numpy to Eigen format

    A_eigen = numpy_to_eigen(A_numpy)

    Parameters
    ----------
    A : Any numpy.ndarray
        Write down the default flags and dtype of a numpy array
    dtype : np.dtype
        Type of the array to return

    Returns
    -------
    A_eigen : numpy.ndarray
        Another numpy ndarray that is compatible with Eigen
    
    Note
    ----
    This is the mapping that is most often used
    Eigen::MatrixXd - np.float64
    Eigen::MatrixXi - np.int32

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------

    .. [1] Shannon, Claude E. "Communication theory of secrecy systems."
    Bell Labs Technical Journal 28.4 (1949): 656-715

    """
    # TODO Check/Decide on if we need to return a view into an array or copy the array directly
    return np.asfortranarray(A, dtype=dtype)

