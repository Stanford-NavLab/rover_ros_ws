import numpy as np
#import math
#from scipy.stats.distributions import chi2
#import sys #include for error messages

class pZ:
    """
    TODO
    """
    def __init__(self, center, generators, covariance) -> None:
        
        #if center.shape[1] != 1:
        #    sys.exit("Check dimension of zonotope center");

        #sTODO: check if generator rows = dim, check if covariance matrix is square dim x dim

        self.dim = center.shape[0]
        self.c = center
        
        self.G = generators
        #if generator matrix is empty, then set empty matrix
        #if isinstance(generators,list):
        #    self.G = np.zeros((self.dim,0));

        self.Sigma = covariance
        #if covariance matrix is empty, then set zero matrix
        #if isinstance(covariance,list):
        #    self.Sigma = np.zeros((self.dim,self.dim));
        pass


    def linear_transform(T, pZ1):
        """
        TODO
        """
        # Calculate new center, generators and covariance matrix
        center = T @ pZ1.c
        generators = T @ pZ1.G
        covariance = T @ pZ1.Sigma @ np.transpose(T)

        return pZ(center, generators, covariance)


    def minkowski_sum(pZ_list):
        """
        TODO
        """
        n = len(pZ_list)
        # Check if sufficient inputs for minkowski sum operation
        #if n < 2:
        #    sys.exit("Need more than one input to perform Minkowski sum");

        #sTODO: check if all inputs are pZ, and check dimensions
        dim = pZ_list[0].c.shape[0]
        center = np.zeros((dim,1))
        generators = np.zeros((dim,0))
        covariance = np.zeros((dim,dim))
        
        # Update center, generators, covariance
        for i in range(n):
            center += pZ_list[i].c
            generators = np.concatenate((generators, pZ_list[i].G), axis=1)
            covariance += pZ_list[i].Sigma

        return pZ(center, generators, covariance)


    def reduce(pZ1, order):
        """
        TODO
        """
        n_generators = pZ1.G.shape[1];        
        # Return if zonotope order is less than desired order
        if ( n_generators/pZ1.dim ) <= order:
            return pZ1

        # Number of highest magnitude generators to get from original pZ
        old_G_n = pZ1.dim*order - pZ1.dim

        # Indices of generator norms in ascending order 
        G_norm_idx = np.argsort(np.linalg.norm( pZ1.G ,axis=0))

        # Interval approximation of lowest magnitude generators
        approx_G_n = pZ1.G.shape[1] - old_G_n
        approx_G = pZ1.G[:,G_norm_idx[:approx_G_n]]

        # Approximate lowest magnitude generators
        interval_G = np.diag((np.abs(approx_G)@np.ones((approx_G_n,1))).transpose()[0])
        
        # Get new generator matrix
        new_G = np.concatenate((pZ1.G[:,G_norm_idx[approx_G_n:]], interval_G), 1)

        return pZ(pZ1.c, new_G, pZ1.Sigma)


    def delete_zeros(pZ1):
        """
        TODO
        """
        new_G = pZ1.G[:,~np.all(pZ1.G == 0, axis = 0)]
        return pZ(pZ1.c, new_G, pZ1.Sigma)


    def conf_zonotope(pZ1, conf_value):
        """
        TODO
        """        
        # Get eigenvalues and eigenvectors
        d, v = np.linalg.eig(pZ1.Sigma)
        
        # Get the confidence value for scaling the ellipsoid
        #conf_value = np.sqrt(chi2.ppf(math.erf(m/np.sqrt(2)),df=pZ1.dim));
        
        # Scale ellipsoid and approximate it with generators
        cov_G = conf_value*(v.T*(np.sqrt( np.abs(d) ))[:,None]).T
        #cov_G = conf_value*(v.T*(np.sqrt(d))[:,None]).T;

        # Add ellipsoid generator to existing generators
        new_G = np.concatenate((pZ1.G, cov_G), 1)

        return pZ( pZ1.c, new_G, np.zeros((pZ1.c.shape[0],pZ1.c.shape[0])) );