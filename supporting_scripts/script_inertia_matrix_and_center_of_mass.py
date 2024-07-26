import numpy as np

def compute_dh(a, alpha, d, theta):
    """
    a: link lengths
    alpha: link twists
    d: link offsets
    theta: joint angles
    This function compute transformation matrices using DH parameters.
    """
    
    # Check if the length of the parameters are equal
    if len(a) != len(alpha) or len(a) != len(d) or len(a) != len(theta):
        raise ValueError("Length of the parameters are not equal")
    
    # Compute transformation matrices
    T = []
    for i in range(len(a)):
        ct = round(np.cos(theta[i]),5)
        st = round(np.sin(theta[i]),5)
        ca = round(np.cos(alpha[i]),5)
        sa = round(np.sin(alpha[i]),5)
        
        # DH transformation matrix
        Ti = np.array([[ct, -st*ca, st*sa, a[i]*ct],
                    [st, ct*ca, -ct*sa, a[i]*st],
                    [0, sa, ca, d[i]],
                    [0, 0, 0, 1]])
        
        T.append(Ti)
    return T

def print_dh(T):
    """
    T: transformation matrices
    This function print transformation matrices.
    """
    # Print transformation matrices
    for i in range(len(T)):
        print(f"T{i+1} = ")
        print(T[i])
        print()


def compute_fk(T,link_number):
    """
    T: transformation matrix
    link_number: link number
    This function compute forward kinematics as: T0n = T01*T12*T23*...*Tn-1n
    """

    temp = []
    if link_number < 0:
        raise ValueError("Link number should be greater than 0")
    if link_number == 0:
        temp = T[0]
    for i in range(link_number):
        if i == 0:
            temp = T[0]@T[1]
        else:
            temp = temp@T[i+1]
    return temp

def calculate_center_of_mass(point,link_number,T):
    """
    point: point in link frame
    link_number: link number
    T: transformation matrix
    This function transform center of mass described in link frame 0 to the reference frame of the i-th link. The formula for the transformation is:

    p_i = T_inverted * p_0
    where:
    p_i: center of mass in i-th link frame
    T_inverted: [R_i^T -R_i^T*p_0
                 0^T         1]
    p_0: center of mass in link frame 0
    """
    
    temp = compute_fk(T,link_number)
    #extracting rotation matrix and origin from transformation matrix
    rotation = temp[0:3,0:3]
    origin = np.atleast_2d(temp[0:3,3]).T

    #computing inverse transformation matrix
    temp_product1 = rotation.transpose()
    temp_product2 = (-rotation.transpose()@origin)
    
    T_inverted = np.concatenate((temp_product1,temp_product2),axis=1)
    T_inverted = np.concatenate((T_inverted,np.array([[0,0,0,1]])),axis=0)
    
    return T_inverted@point


def compute_inertia_matrix(T,I_tool,link_number):
    """
    T: transformation matrix
    I_tool: inertia matrix
    link_number: link number

    Il tool ci restituisce la matrice di inerzia riferita al sistema di riferimento del link 0, di conseguenza dobbiamo trasformarla nel sistema di riferimento del link i-esimo. La formula per la conversione è la seguente:
    Ii = R_i * I_tool * R_i^T
    Nota: R_i è la matrice di rotazione che trasforma il sistema di riferimento del link 0 nel sistema di riferimento del link i-esimo.
    Dato che dalle matrici di trasformazione omogenea T_i possiamo estrarre la matrice di rotazione R_i^T (rotation) la formula diventa:
    Ii = rotation^T* I_tool * rotation
    """

    temp = compute_fk(T,link_number)
    rotation = temp[0:3,0:3]
    print("rotation\n",rotation)
    print("I_tool\n",I_tool)
    return rotation.T@I_tool@rotation

if __name__ == "__main__":


    #### Parameters definition ####


    # Define DH parameters
    a = [0.15, 0.79, 0.15,0,0,0]  # Link lengths
    alpha = [np.pi/2, 0, np.pi/2,-np.pi/2,np.pi/2,0]  # Link twists
    d = [0.525,0, 0,0.86,0,0.1]  # Link offsets
    theta = [0, np.pi/2,0,0,0,0]  # Joint angles

    # Define an array of center of mass points defined in link frame 0. Note that we need to calculate center of mass for only link 2,3,4,5
    point_array = np.array([[0.14969283,-0.15736359,0.87765022,1],[0.20590633,0.03907458,1.3758284,1],[7.3466118000e-01,-1.3092480000e-02, 1.4650871300e+00,1],[1.0125965100e+00,1.9908600000e-03,1.4610867300e+00,1]]) # da 2

    # Define inertia matrix for link 2
    I2 = np.array([[2.9174838317, 1.1574578719e-03,-7.1824213731e-04],
                [1.1574578719e-03, 2.9473440414e+00,  2.1914272161e-03],
                [-7.1824213731e-04, 2.1914272161e-03, 2.2868478112e-01]])
    
    # Define inertia matrix for link 3
    I3 = np.array([[7.1361497902e-01, 4.6909979651e-02,-8.8835555704e-02],
                [4.6909979651e-02, 8.0327105899e-01,  -5.4618469458e-03 ],
                [-8.8835555704e-02, -5.4618469458e-03 , 5.9966985758e-01]])

    # Define inertia matrix for link 4
    I4 = np.array([[8.8216498481e-02, 3.4660467838e-02,3.3249136365e-04],
                [3.4660467838e-02, 7.9006202872e-01,  -2.9929194141e-05 ],
                [3.3249136365e-04, -2.9929194141e-05 , 8.1628423106e-01 ]])

    # Define inertia matrix for link 5
    I5 = np.array([[4.6430664263e-03, 7.9505177901e-05,-1.5290186582e-04],
                [7.9505177901e-05, 1.2118631312e-02,  -1.5730265574e-05 ],
                [-1.5290186582e-04, -1.5730265574e-05, 1.2038816810e-02 ]])
    
    inertia_matrixs = [I2,I3,I4,I5]


    #### Main ####

    # Compute transformation matrices using DH parameters
    T = compute_dh(a, alpha, d, theta)
    
    # Print transformation matrices
    print_dh(T)
    
    # calculate center of mass for link 2,3,4,5 as described in the calculate_center_of_mass function
    calculate_center_of_mass(point_array[0],0,T)
    for i in range(point_array.shape[0]):
        center_of_mass = calculate_center_of_mass(point_array[i],i,T)
        print("center of mass link ",i+2," : ",center_of_mass)

    # compute inertia matrix for link 2,3,4,5 as described in the compute_inertia_matrix function
    for i in range(len(inertia_matrixs)):
        inertia_matrix = compute_inertia_matrix(T,inertia_matrixs[i],i+2)
        print("inertia matrix link ",i+2,":\n",inertia_matrix)

