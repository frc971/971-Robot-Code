'''Returns a select which is either srcs or an empty main function.'''
def empty_main_if_asan(srcs):
  return select({
    '//conditions:default': srcs,
  })
