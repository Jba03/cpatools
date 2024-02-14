#ifndef fileformat_hh
#define fileformat_hh

#include "types.hh"
#include "memory.hh"
#include <vector>

namespace CPA {
  
  namespace FileFormat {
    
    /// GCN Texture Palette Library
    struct TPL {
      struct FileHeader {
        uint32 code;
        uint32 imageCount;
        uint32 imageTableOffset;
      };
      
      struct PaletteHeader {
        uint16 count;
        uint16 unknown;
        uint32 format;
        uint32 paletteDataOffset;
      };
      
      struct ImageHeader {
        uint16 height;
        uint16 width;
        uint32 format;
        uint32 imageDataOffset;
        uint32 wrapS;
        uint32 wrapT;
        uint32 minFilter;
        uint32 magFilter;
        float32 lodBias;
        uint8 edgeLodEnable;
        uint8 minLod;
        uint8 maxLod;
        uint8 unpacked;
      };
      
      static bool isTPL(Stream& s) {
        return s.read<FileHeader, true>().code == 0x0020AF30;
      }
      
      TPL(Stream& s);
    };
    
    enum DataFileIndex {
      FIX = 0,
      LVL = 1,
      Transit = 2,
      Transit2 = 3,
      VertexBuffer = 4,
      KeyFramesFIX = 5,
      KeyFramesLVL = 6,
    };
    
    /// Pointer file
    template <typename FileIndex = DataFileIndex>
    struct PTR {
      PTR(Stream& s);
      /// <SourceFile, SourcePointer>
      using PointerFilePair = std::tuple<FileIndex, pointer<>>;
      /// <SourceFile, SourcePointer, TargetFile, TargetPointer>
      using FillInPointerFilePair = std::tuple<FileIndex, pointer<>, FileIndex, pointer<>>;
      /// Default pointers
      std::vector<PointerFilePair> pointers;
      /// Pointers to be replaced in the level
      std::vector<FillInPointerFilePair> fillInPointers;
    };
    
  };
};

#endif /* fileformat_hh */
