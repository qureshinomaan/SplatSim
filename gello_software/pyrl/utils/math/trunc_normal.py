import numpy as np


def trunc_normal(shape, a=-2, b=2):
    from scipy.stats import norm
    """
    :param shape: The shape of the trunc normal
    :param a, b: Sample between [a, b] with i.i.d. normal distribution
    :return: samples
    Gaussian density of N(mu, sigma^2): exp(-((x - mu) / sigma)^2 / 2) / (sqrt(2 * pi) * sigma)
    """
    a_cdf = norm.cdf(a)
    b_cdf = norm.cdf(b)
    p = a_cdf + (b_cdf - a_cdf) * np.random.rand(*shape)
    return np.clip(norm.ppf(p), a, b)
