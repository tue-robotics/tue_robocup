import numpy as np


# Visualizing convolution
def iter_pixels(image):

    # Yielding pixel position (row, column) and pixel intensity
    height, width = image.shape[:2]
    for i in range(height):
        for j in range(width):
            yield (i, j), image[i, j]


# Adding zero-padding
def padding_for_kernel(kernel):

    # Slicing to ignore RGB channels, if they exist
    image_shape = kernel.shape[:2]

    # Only handle kernels with odd dimensions
    assert all((size % 2) == 1 for size in image_shape)

    # Returning the amount of padding needed for each side of an image
    return [(size - 1) // 2 for size in image_shape]


def add_padding(image, kernel):
    h_pad, w_pad = padding_for_kernel(kernel)
    return np.pad(image, ((h_pad, h_pad), (w_pad, w_pad)),
                  mode='constant', constant_values=0)


def remove_padding(image, kernel):

    # Creating a 2D slice for grabbing the inner image region
    inner_region = []
    for pad in padding_for_kernel(kernel):
        slice_i = slice(None) if pad == 0 else slice(pad, -pad)
        inner_region.append(slice_i)
    return image[tuple(inner_region)]


# Slicing windows
def window_slice(center, kernel):
    r, c = center
    r_pad, c_pad = padding_for_kernel(kernel)
    return [slice(r - r_pad, r + r_pad + 1), slice(c - c_pad, c + c_pad + 1)]


# Applying convolution kernel to image patch
def apply_kernel(center, kernel, original_image):
    image_patch = original_image[tuple(window_slice(center, kernel))]
    return np.sum(kernel * image_patch)


# Moving kernel over the image
def iter_kernel_labels(image, kernel):
    original_image = image
    image = add_padding(original_image, kernel)
    i_pad, j_pad = padding_for_kernel(kernel)

    for (i, j), pixel in iter_pixels(original_image):

        # Shifting the center of the kernel to ignore padded border
        i += i_pad
        j += j_pad
        mask = np.zeros(image.shape, dtype=int)
        mask[tuple(window_slice((i, j), kernel))] = kernel
        yield (i, j), mask


# Visualizing kernel as it moves over the image
def visualize_kernel(kernel_labels, image):
    return kernel_labels + image


# Full process of applying image convolution
def convolution_full(ax, image, kernel, **kwargs):
    gen_kernel_labels = iter_kernel_labels(image, kernel)
    image_cache = []
    image_padded = add_padding(image, kernel)

    # Plotting filtered image
    for i_step in range(image.size - 1):
        filtered_prev = image_padded if i_step == 0 else image_cache[-1][1]
        filtered = filtered_prev.copy()

        # Getting the labels used to visualize the kernel
        center, kernel_labels = next(gen_kernel_labels)

        # Modifying the pixel value at the kernel center
        filtered[center] = apply_kernel(center, kernel, image_padded)

        # Taking the original image and overlaying the kernel visualization
        kernel_overlay = visualize_kernel(kernel_labels, image_padded)
        image_cache.append((kernel_overlay, filtered))

    # Removing padding that was added to deal with boundary conditions
    image_triple = [remove_padding(each, kernel)
                    for each in image_cache[i_step]]
    image_triple.insert(1, kernel)

    # Sending filtered image to 'visual_servoing.py'
    ax.imshow(image_triple[2], aspect='auto', **kwargs)
