"""test module for module Publish in PyPI
   this is the print list module for testing
   it can auto print the list elements,
   if the sub element is a list, it can auto indent
"""
def print_list(the_list,indent=False,level=0):
    for each_item in the_list:
        if isinstance(each_item,list):
            print_list(each_item, indent, level+1)
        else:
            if indent:
                for tab_stop in range(level):
                    print("\t",end='')
            print(each_item)