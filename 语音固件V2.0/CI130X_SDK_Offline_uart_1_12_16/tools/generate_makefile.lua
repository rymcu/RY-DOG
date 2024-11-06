


function get_file_name(file_path)
    --获取文件名（包含扩展名）
    local name = string.match(file_path, ".+\\([^\\]*%.%w+)$")
    local ext_name
    if name == nil then 
        name = string.match(file_path, ".+/([^/]*%.%w+)$")
    end
    if name then 
        --获取扩展名，文件名（不包含扩展名）
        ext_name = name:match(".+%.(%w+)$")
        if ext_name then
            name = name:sub(1, name:len()-ext_name:len() - 1)
        end
    end
    return name, ext_name
end

local ret = lfs.attributes("build");
if ret == nil then
    lfs.mkdir("build")
end

ret = lfs.attributes("build/objs");
if ret == nil then
    lfs.mkdir("build/objs")
end

local output_file_path = "build/source_file.mk"
local  output_file = io.open(output_file_path, "w")
assert(output_file ~= nil, "open file:"..output_file_path.." failed")

output_file:write("# This file is maked by run generate_makefile.lua\n")

local handle_line = {
    ["c"] = function(line, name, ext_name)
        local obj_path = "build/objs/"..name..".o"
        output_file:write("OBJS += "..obj_path.."\n" )
        output_file:write("-include build/objs/"..name..".d\n")
        output_file:write(obj_path.." : $(ROOT_DIR)/"..line.."\n")
        output_file:write("\t$(CC_PREFIX)$(CC) $(C_FLAGS) -c -o \"$@\" \"$<\"".."\n\n")
        print("c file: "..line)
    end,
    ["s"] = function(line, name, ext_name)
        local obj_path = "build/objs/"..name..".o"
        output_file:write("OBJS += "..obj_path.."\n" )
        output_file:write("-include build/objs/"..name..".d\n")
        output_file:write(obj_path.." : $(ROOT_DIR)/"..line.."\n")
        output_file:write("\t$(CC_PREFIX)$(AS) $(S_FLAGS) -c -o \"$@\" \"$<\"".."\n\n")
        print("s file: "..line)
    end,
    ["a"] = function(line, name, ext_name)
        output_file:write("LIB_FILES += $(ROOT_DIR)/"..line.."\n")
        output_file:write("LD_FLAGS += -L$(ROOT_DIR)/"..line:sub(1, line:len() - 1 - name:len() - 1 - ext_name:len()).."\n")
        output_file:write("LIBS += -l"..name:sub(4, -1).."\n")
        print("a file: "..line)
    end,
    ["inc_path"] = function(line)
        output_file:write("C_FLAGS += -I$(ROOT_DIR)/"..line.."\n")
        print("h path: "..line)
    end
}

for line in io.lines(arg[1]) do
    line = string.gsub(line, "^[%s]*(.-)[%s]*$", "%1")       --去除前后空字符
    
    if string.sub(line, 1, 2) == "//" then
        print('comment:'..line)
    elseif line == "" then
        print("..."..line)
    elseif line:sub(1, 12) == "source-file:" then
        line = line:sub(14, -1)
        local name, ext_name = get_file_name(line)
        if ext_name ~= nil then
            handle_line[string.lower(ext_name)](line, name, ext_name)
        end
    elseif line:sub(1, 13) == "library-file:" then
        line = line:sub(15, -1)
        local name, ext_name = get_file_name(line)
        if ext_name ~= nil then
            handle_line[string.lower(ext_name)](line, name, ext_name)
        end
    elseif line:sub(1, 13) == "include-path:" then
        line = line:sub(15, -1)
        handle_line['inc_path'](line)
    end
end

output_file:close()


