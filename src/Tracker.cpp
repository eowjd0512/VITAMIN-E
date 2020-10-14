/* #include "Tracker.h"

namespace VITAMINE
{

    void Tracker::track()
    {
                Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        imshow("camera1", frame);  
        imshow("dd", gray);
  
        if (waitKey(20) >= 0) break;  
    }

    void Tracker::tracking(){

        //TODO: generate frame

        //TODO: feature track

        
        db = []
        matcher = Matcher()
        #src = os.path.expanduser('~/Videos/VID_20190327_194904.mp4')
        src = 0
        #cam = cv2.VideoCapture(0)
        cam = cv2.VideoCapture(src)
        img = None
        cv2.namedWindow('img', cv2.WINDOW_NORMAL)
        #cv2.namedWindow('dbg', cv2.WINDOW_NORMAL)

        trk  = None
        path = []
        iter = 0
        scale = 1.#/4
        while (true){
            ret, img = cam.read(img)
            if not ret:
                break
            img = cv2.resize(img, None, fx=scale, fy=scale)
            #lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            #kappa = curvature(lab * (100.0/255.0/255.0, 1.0/255.0,1.0/255.0))
            kappa = curvature(img/255.0)
            #kappa = normalize(np.abs(kappa), axis=(0,1))
            kappa = np.linalg.norm(kappa, axis=-1)
            #cv2.imshow('kappa', normalize(kappa))
            maxima, idx = local_maxima(kappa)
            #cv2.imshow('maxima', kappa)
            db.append( (img, idx) )

            if len(db) >= 2:
                mdata, dbg = matcher.match(db[-2], db[-1])
                T_A, T_b = get_dominant_motion(mdata)
                if trk is None:
                    trk  = db[-2][1]
                    path = trk[None,:]
                    cols = np.random.uniform(0, 255, (len(trk),3))
                trk, good = vitatrack(trk, kappa, T_A, T_b)
                #trk, good = lktrack(db[-2][0], db[-1][0], trk)
                path = path[:, good]
                cols = cols[good]
                path = np.append(path, trk[None,:], axis=0)

                tmp = img.copy()
                for p, c in zip(path.swapaxes(0,1)[...,::-1], cols):
                    cv2.polylines(tmp,
                            #path.swapaxes(0,1)[...,::-1],
                            p[None,...],
                            False, c
                            )
                img = cv2.addWeighted(img, 0.75, tmp, 0.25, 0.0)
                for p, c in zip(trk,cols):
                    cv2.circle(img, (p[1], p[0]), 2, c)

                #cv2.imshow('dbg', dbg)

            #plt.hist(kappa.ravel(), bins='auto')
            #plt.pause(0.001)

            #print viz.min(axis=(0,1)), viz.max(axis=(0,1))
            #print kappa.min(axis=(0,1)),  kappa.max(axis=(0,1))
            #plt.hist(kappa.ravel())
            #plt.pause(0.001)

            luv = cv2.cvtColor(img, cv2.COLOR_BGR2LUV)
            luv[..., 2] = 128
            img2 = cv2.cvtColor(luv, cv2.COLOR_LUV2BGR)
            viz = np.clip(img/255.+maxima, 0.0, 1.0)
            #viz = cv2.addWeighted(img, 0.2, kappa, 255./0.8, 0.0,
            #        dtype=cv2.CV_8U)
            #viz = np.concatenate( (img, img2), axis=1)
            #viz = kappa - kappa.min(axis=(0,1),keepdims=True)
            cv2.imwrite('/tmp/frame{:04d}.png'.format(iter), (viz*255).astype(np.uint8) )
            cv2.imshow('img', viz)
            k = cv2.waitKey(1)
            if k in [27, ord('q')]:
                break
            iter += 1
        }
    }

} */