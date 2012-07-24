import numpy as np
import time
import matplotlib.pyplot as plt

def match(template, search, return_error=False):
    # should both be odd sized
    #t_start = time.time()
    
    r = int((len(template)-1)/2)
    err = np.zeros(len(search)-r*2)
    
    for p in range(r, len(search)-r):
        diff = search[p-r:p+r+1] - template
        err[p-r] = np.sum(np.abs(diff))*np.sign(np.sum(diff))
    
    if return_error:
        #print 'calculation time: ', time.time()-t_start
        return err
        
    else:
        shift = np.argmin(np.abs(err)) - (len(err)-1)/2
        #print 'calculation time: ', time.time()-t_start
        return shift
        
def smoother(a, n):
    a_smooth = np.zeros_like(a)
    for i, v in enumerate(a[n:-n]):
        a_smooth[i] = np.mean(a[i-n:i+n])
    return a_smooth
        
        
def calc_pixel_shifts_per_pixel(image0, image1, template_radius, search_radius, resize=None):
    if resize is not None:
        new_pixels = np.linspace(0,len(image0),resize)
        old_pixels = np.linspace(0,len(image0),len(image0))
        image0 = np.interp(new_pixels, old_pixels, image0)
        image1 = np.interp(new_pixels, old_pixels, image1)
        
    image0 = smoother(image0, 2)
    image1 = smoother(image1, 2)
                
    pixel_shifts = np.zeros(len(image1))
    i = search_radius
    for p in range(search_radius, len(image1)-search_radius):
        template = image0[p-template_radius:p+template_radius+1]
        search = image1[p-search_radius:p+search_radius+1]
        pixel_shifts[i] = match(template, search)
        i += 1
        
    if resize is not None:
        factor = len(new_pixels) / float(len(old_pixels))
        print factor
        #pixel_shifts = np.interp(old_pixels, new_pixels, pixel_shifts)#*factor
        
    return pixel_shifts

###########################################################################################
 
def calc_diffs_for_shift(template, search, shift):
    errs = np.zeros_like(search, dtype=np.float32)
    if shift > 0:
        errs[0,0:search.shape[1]-shift] = search[0,0:search.shape[1]-shift] - template[0,shift:]
    elif shift < 0:
        errs[0,-1*shift:search.shape[1]] = search[0,-1*shift:search.shape[1]] - template[0,0:shift]
    elif shift == 0:
        errs = search - template
    return errs
    

def calc_all_shifts(template, search, shift_radius, plot=False):
    shifts = np.zeros((shift_radius*2, search.shape[1]))
    i = 0
    for shift in range(-1*shift_radius, shift_radius):
        tmp = calc_diffs_for_shift(template, search, shift)
        shifts[i,:] = tmp
        i += 1
    
    if plot:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for i in range(shifts.shape[0]):
            ax.plot(shifts[i,:])
        plt.show()
    
    return shifts
    

            
def calc_pixel_shifts(template, search, template_radius, search_radius):
    #template = template - np.mean(template)
    #search = search - np.mean(search)
    
    resolution = 'subpixel'
    
    if resolution == 'subpixel':
        shifts = calc_all_shifts(template, search, search_radius)
        shift_radii = np.arange(-1*search_radius, search_radius)
        pixel_shifts = np.zeros_like(search)
        for p, i in enumerate(pixel_shifts[0,:]):
            shift_slice = shifts[:,p-template_radius:p+template_radius+1]
            shift_slice_mean = np.mean( np.abs(shift_slice), axis=1)
            shift_slice_signs = np.mean( np.sign(shift_slice), axis=1)
            
            shift_index_guess = np.argmin( shift_slice_mean )

            try:
                shift_radius = np.interp(0, (shift_slice_mean*shift_slice_signs)[shift_index_guess-1:shift_index_guess+2], shift_radii[shift_index_guess-1:shift_index_guess+2])
            except:
                shift_index = shift_index_guess
                shift_radius = shift_radii[shift_index]
                
            pixel_shifts[0,p] = shift_radius
            
    if resolution == 'pixel':
        shifts = calc_all_shifts(template, search, search_radius)
        shift_radii = np.arange(-1*search_radius, search_radius)
        pixel_shifts = np.zeros_like(search)
        for p, i in enumerate(pixel_shifts[0,:]):
            shift_slice = np.abs( shifts[:,p-template_radius:p+template_radius+1] )
            shift_slice = np.mean(shift_slice, axis=1)
            shift_index = np.argmin( shift_slice )
            pixel_shifts[0,p] = shift_radii[shift_index]
        
    return pixel_shifts
    

def reichardt_correlator(image0, image1, r):
    image0 = image0 - np.mean(image0)
    image1 = image1 - np.mean(image1)
        
    def two_pixel_delay_and_correlate(px_delayed, px_now):
        # px_ are 2-pixel arrays
        #print px_now
        correlated_0 = px_now[0]*px_delayed[-1]
        correlated_1 = px_now[-1]*px_delayed[0]
        return correlated_1-correlated_0
    
    correlated = np.zeros_like(image1)
    for i, c in enumerate(correlated[0,0:-r]):
        c = two_pixel_delay_and_correlate(image0[0,i:i+r], image1[0,i:i+r])
        correlated[0,i] = c
    return correlated
    
    
    
    
        
if __name__ == '__main__':
    
    
    if 0:
        npixels = 659
        ashift = -2
        n = np.linspace(0,np.pi*2,npixels)
        n2 = np.linspace(0,np.pi*2*10,npixels)
        
        b = np.zeros((1,npixels))
        a = np.zeros((1,npixels))
        
        b[0,:] = np.sin(n+n2)
        
        if ashift >= 0:
            a[0,0:-1*ashift] = b[0,ashift:]
        else:
            a[0,-1*ashift:] = b[0,0:ashift]
        
    if 1:
        image_file = '/home/floris/Desktop/american-dipper-zion-waterfall-20120602_0787.jpg'
        image = plt.imread(image_file)
        image = image[50,:,0]
        npixels = len(image)
        
        b = np.zeros((1,npixels))
        b[0,:] = image
        
        
        ashift = 3
        
        a = np.zeros((1,npixels))
        
        if ashift >= 0:
            a[0,0:-1*ashift] = b[0,ashift:]
        else:
            a[0,-1*ashift:] = b[0,0:ashift]
            
            
        
    time_start = time.time()
    pixel_shifts = calc_pixel_shifts(a, b, 5, 15)
    print time.time() - time_start

    
    
    
