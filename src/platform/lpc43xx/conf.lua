-- Configuration file for the LPC43xx backend

addi( sf( 'src/platform/%s/drivers/inc', platform ) )
addi( sf( 'src/platform/%s/usbstack/inc', platform ) )

local fwlib_files = utils.get_files( sf( "src/platform/%s/drivers/src", platform ), ".*%.c$" )
-- fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/usbstack/src", platform ), ".*%.c$" )
specific_files = "cr_startup_lpc43xx.c sysinit.c platform.c"

local board = comp.board:upper()

local ldscript = "LPC4367.ld"

-- Prepend with path
specific_files = fwlib_files .. " " .. utils.prepend_path( specific_files, sf( "src/platform/%s", platform ) )
specific_files = specific_files .. " src/platform/cortex_utils.s src/platform/arm_cortex_interrupts.c"
ldscript = sf( "src/platform/%s/%s", platform, ldscript )

addm{ "FOR" .. comp.cpu:upper(), 'gcc', 'CORTEX_M4' }

-- Standard GCC flags
addcf{ '-DCORE_M4 -DNO_BOARD_LIB' }
addcf( { '-g' } )
delcf( { '-Os' } )
delcf( { '-O1' } )
delcf( { '-O2' } )
delcf( { '-O3' } )
addcf( { '-ffunction-sections' } )
addcf( { '-fdata-sections' } )
addcf( { '-fno-strict-aliasing', '-Wall' } )
addlf{ '-Wl,--gc-sections', }
addlf{ '-nostartfiles', '-nostdlib', '-T', ldscript, '-Wl,--allow-multiple-definition' }
addaf{ '-x', 'assembler-with-cpp', '-Wall' }
addlib{ 'c','gcc','m' }

local target_flags = { '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16' }

-- Configure general flags for target
addcf{ target_flags, '-mlittle-endian' }
addlf{ target_flags, '-Wl,-e,ResetISR', '-Wl,-static' }
addaf{ target_flags }

-- Toolset data
tools.lpc43xx = {}

-- Array of file names that will be checked against the 'prog' target; their absence will force a rebuild
tools.lpc43xx.prog_flist = { output .. ".bin" }

-- We use 'gcc' as the assembler
toolset.asm = toolset.compile

