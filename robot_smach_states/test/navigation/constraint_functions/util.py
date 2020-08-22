from numpy import linspace


def constraint_strings_equal(constraint, verification):
    """
    check that two constraint strings lead to the same result.
    We cannot simply do string comparison as we want to verify that the underlying mathematical expression is identical
     rather than the exact string
    :param constraint: string containing the first constraint
    :param verification: string containing the second constraint
    :return: whether or not the constraints are equal.
    """
    constraint_string = constraint.replace("^", "**")
    verification_string = verification.replace("^", "**")
    for x in linspace(-5, 5, 100):
        for y in linspace(-5, 5, 100):
            if eval(constraint_string) != eval(verification_string):
                return False, "For point ({}, {}), constraint {} resolved to {}, expected {}"\
                    .format(x, y, constraint_string, eval(constraint_string), eval(verification_string))
    return True, ""
