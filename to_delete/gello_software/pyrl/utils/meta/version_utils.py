def digit_version(version_str):
    ret = []
    for x in version_str.split("."):
        if x.isdigit():
            ret.append(int(x))
        elif x.find("rc") != -1:
            ret = x.split("rc")
            ret.append(int(patch_version[0]) - 1)
            ret.append(int(patch_version[1]))
    return tuple(ret)
