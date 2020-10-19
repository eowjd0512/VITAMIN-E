/* #include "VitamineFunction.h"

namespace VITAMINE{

    const void VitaminFunction::vitatrack(){
        pt0_ = kpt0[:, ::-1].astype(np.float32) # (i,j) -> (x,y)
        pt1_ = pt0_.dot(T_A.T) + T_b # prediction from dominant motion

        pt1  = np.round(pt1_[:,::-1]).astype(np.int32) # back to (i,j) order
        good = np.logical_and.reduce([
            pt1[:,0] >= 0,
            pt1[:,0] < kappa.shape[0],
            pt1[:,1] >= 0,
            pt1[:,1] < kappa.shape[1],
            ])
        pt1_ = pt1_[good]
        pt1  = pt1[good]
        F    = kappa[pt1[:,0], pt1[:,1]]
        idx  = np.arange(len(pt1))

        while True:
            #break
            msk, pt1_d, F = hill_climb(kappa, pt1[idx], pt1_[idx], F, lmd=lmd)
            if np.sum(msk) <= 0:
                break
            idx = idx[msk]
            pt1[idx] = pt1_d

        //TODO: Frame
        
        return pt1, good
    }

    
    const void VitaminFunction::p_fn(){
        # really should be `rho`, but using p anyway
        # Geman-McClure Kernel
        xsq = np.square(x)
        ssq = np.square(sigma)
        return xsq / (xsq + ssq)
    }
    const void VitaminFunction::w_fn(){
        return 1.0 - p_fn(x, sigma=sigma)
    }
    
    const void VitaminFunction::get_dominant_motion(){
        pt0, pt1, m01 = mdata
        i0, i1 = np.stack([(m.queryIdx, m.trainIdx) for m in m01], axis=1)
        #least_squares(cost_fn,
        #cv2.estimateRigidTransform(pt0[i0], pt1[i1], True)
        M, _ = cv2.estimateAffine2D(pt0[i0], pt1[i1])
        A, b = M[:, :2], M[:, 2]
        return A, b
    }
    const void VitaminFunction::hill_climb(){
        kappa_pad = np.pad(kappa, ((1,1),(1,1)),
            mode='constant', constant_values=-np.inf)

        Fs = []
        ds = []
        for di in [-1,0,1]:
            for dj in [-1,0,1]:
                ds.append( (di,dj) )
                if di==0 and dj==0:
                    Fs.append(F)
                    continue
                d_pt = np.linalg.norm(pt1 + [di,dj] - pt1_, axis=-1)
                f = kappa_pad[pt1[:,0]+(1+di), pt1[:,1]+(1+dj)] + lmd * w_fn(d_pt)
                Fs.append(f)

        Fs=np.float32(Fs)
        ds=np.int32(ds)

        sel = np.argmax(Fs, axis=0)
        msk = (sel != 4)
        pt1_out = pt1[msk] + ds[sel[msk]]
        F1  = np.max(Fs,axis=0)[msk]

        return msk, pt1_out, F1
    }

} */