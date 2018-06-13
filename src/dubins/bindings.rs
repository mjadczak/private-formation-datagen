/* automatically generated by rust-bindgen */

pub const EDUBOK: u32 = 0;
pub const EDUBCOCONFIGS: u32 = 1;
pub const EDUBPARAM: u32 = 2;
pub const EDUBBADRHO: u32 = 3;
pub const EDUBNOPATH: u32 = 4;

#[repr(C)]
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum DubinsPathType {
    LSL,
    LSR,
    RSL,
    RSR,
    RLR,
    LRL,
}

impl Default for DubinsPathType {
    fn default() -> Self {
        DubinsPathType::LSL
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default, Serialize, Deserialize)]
pub struct DubinsPath {
    pub qi: [f64; 3usize],
    pub param: [f64; 3usize],
    pub rho: f64,
    pub type_: DubinsPathType,
}
#[test]
fn bindgen_test_layout_DubinsPath() {
    assert_eq!(
        ::std::mem::size_of::<DubinsPath>(),
        64usize,
        concat!("Size of: ", stringify!(DubinsPath))
    );
    assert_eq!(
        ::std::mem::align_of::<DubinsPath>(),
        8usize,
        concat!("Alignment of ", stringify!(DubinsPath))
    );
    assert_eq!(
        unsafe { &(*(::std::ptr::null::<DubinsPath>())).qi as *const _ as usize },
        0usize,
        concat!(
            "Offset of field: ",
            stringify!(DubinsPath),
            "::",
            stringify!(qi)
        )
    );
    assert_eq!(
        unsafe { &(*(::std::ptr::null::<DubinsPath>())).param as *const _ as usize },
        24usize,
        concat!(
            "Offset of field: ",
            stringify!(DubinsPath),
            "::",
            stringify!(param)
        )
    );
    assert_eq!(
        unsafe { &(*(::std::ptr::null::<DubinsPath>())).rho as *const _ as usize },
        48usize,
        concat!(
            "Offset of field: ",
            stringify!(DubinsPath),
            "::",
            stringify!(rho)
        )
    );
    assert_eq!(
        unsafe { &(*(::std::ptr::null::<DubinsPath>())).type_ as *const _ as usize },
        56usize,
        concat!(
            "Offset of field: ",
            stringify!(DubinsPath),
            "::",
            stringify!(type_)
        )
    );
}
/// Callback function for path sampling
///
/// @note the q parameter is a configuration
/// @note the t parameter is the distance along the path
/// @note the user_data parameter is forwarded from the caller
/// @note return non-zero to denote sampling should be stopped
pub type DubinsPathSamplingCallback = ::std::option::Option<
    unsafe extern "C" fn(q: *mut f64, t: f64, user_data: *mut ::std::os::raw::c_void)
        -> ::std::os::raw::c_int,
>;
extern "C" {
    /// Generate a path from an initial configuration to
    /// a target configuration, with a specified maximum turning
    /// radii
    ///
    /// A configuration is (x, y, theta), where theta is in radians, with zero
    /// along the line x = 0, and counter-clockwise is positive
    ///
    /// @param path  - the resultant path
    /// @param q0    - a configuration specified as an array of x, y, theta
    /// @param q1    - a configuration specified as an array of x, y, theta
    /// @param rho   - turning radius of the vehicle (forward velocity divided by maximum angular velocity)
    /// @return      - non-zero on error
    #[link_name = "\u{1}_dubins_shortest_path"]
    pub fn dubins_shortest_path(
        path: *mut DubinsPath,
        q0: *mut f64,
        q1: *mut f64,
        rho: f64,
    ) -> ::std::os::raw::c_int;
}
extern "C" {
    /// Generate a path with a specified word from an initial configuration to
    /// a target configuration, with a specified turning radius
    ///
    /// @param path     - the resultant path
    /// @param q0       - a configuration specified as an array of x, y, theta
    /// @param q1       - a configuration specified as an array of x, y, theta
    /// @param rho      - turning radius of the vehicle (forward velocity divided by maximum angular velocity)
    /// @param pathType - the specific path type to use
    /// @return         - non-zero on error
    #[link_name = "\u{1}_dubins_path"]
    pub fn dubins_path(
        path: *mut DubinsPath,
        q0: *mut f64,
        q1: *mut f64,
        rho: f64,
        pathType: DubinsPathType,
    ) -> ::std::os::raw::c_int;
}
extern "C" {
    /// Calculate the length of an initialised path
    ///
    /// @param path - the path to find the length of
    #[link_name = "\u{1}_dubins_path_length"]
    pub fn dubins_path_length(path: *mut DubinsPath) -> f64;
}
extern "C" {
    /// Return the length of a specific segment in an initialized path
    ///
    /// @param path - the path to find the length of
    /// @param i    - the segment you to get the length of (0-2)
    #[link_name = "\u{1}_dubins_segment_length"]
    pub fn dubins_segment_length(path: *mut DubinsPath, i: ::std::os::raw::c_int) -> f64;
}
extern "C" {
    /// Return the normalized length of a specific segment in an initialized path
    ///
    /// @param path - the path to find the length of
    /// @param i    - the segment you to get the length of (0-2)
    #[link_name = "\u{1}_dubins_segment_length_normalized"]
    pub fn dubins_segment_length_normalized(path: *mut DubinsPath, i: ::std::os::raw::c_int)
        -> f64;
}
extern "C" {
    /// Extract an integer that represents which path type was used
    ///
    /// @param path    - an initialised path
    /// @return        - one of LSL, LSR, RSL, RSR, RLR or LRL
    #[link_name = "\u{1}_dubins_path_type"]
    pub fn dubins_path_type(path: *mut DubinsPath) -> DubinsPathType;
}
extern "C" {
    /// Calculate the configuration along the path, using the parameter t
    ///
    /// @param path - an initialised path
    /// @param t    - a length measure, where 0 <= t < dubins_path_length(path)
    /// @param q    - the configuration result
    /// @returns    - non-zero if 't' is not in the correct range
    #[link_name = "\u{1}_dubins_path_sample"]
    pub fn dubins_path_sample(path: *mut DubinsPath, t: f64, q: *mut f64) -> ::std::os::raw::c_int;
}
extern "C" {
    /// Walk along the path at a fixed sampling interval, calling the
    /// callback function at each interval
    ///
    /// The sampling process continues until the whole path is sampled, or the callback returns a non-zero value
    ///
    /// @param path      - the path to sample
    /// @param stepSize  - the distance along the path for subsequent samples
    /// @param cb        - the callback function to call for each sample
    /// @param user_data - optional information to pass on to the callback
    ///
    /// @returns - zero on successful completion, or the result of the callback
    #[link_name = "\u{1}_dubins_path_sample_many"]
    pub fn dubins_path_sample_many(
        path: *mut DubinsPath,
        stepSize: f64,
        cb: DubinsPathSamplingCallback,
        user_data: *mut ::std::os::raw::c_void,
    ) -> ::std::os::raw::c_int;
}
extern "C" {
    /// Convenience function to identify the endpoint of a path
    ///
    /// @param path - an initialised path
    /// @param q    - the configuration result
    #[link_name = "\u{1}_dubins_path_endpoint"]
    pub fn dubins_path_endpoint(path: *mut DubinsPath, q: *mut f64) -> ::std::os::raw::c_int;
}
extern "C" {
    /// Convenience function to extract a subset of a path
    ///
    /// @param path    - an initialised path
    /// @param t       - a length measure, where 0 < t < dubins_path_length(path)
    /// @param newpath - the resultant path
    #[link_name = "\u{1}_dubins_extract_subpath"]
    pub fn dubins_extract_subpath(
        path: *mut DubinsPath,
        t: f64,
        newpath: *mut DubinsPath,
    ) -> ::std::os::raw::c_int;
}
