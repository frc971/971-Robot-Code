def clean_dep(target):
    """Returns string to 'target' in @org_frc971 repository.

    Use this function when referring to targets in the @org_frc971
    repository from macros that may be called from external repositories.

    This is stolen from tensorflow.bzl (https://github.com/tensorflow/tensorflow/blob/69b50ff7537c7e9bea8ad45b973f75e66a9a0fb9/tensorflow/tensorflow.bzl#L102C1-L111C30).
    """

    # A repo-relative label is resolved relative to the file in which the
    # Label() call appears, i.e. @org_frc971.
    return str(Label(target))
