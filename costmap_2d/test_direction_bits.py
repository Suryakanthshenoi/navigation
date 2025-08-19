#!/usr/bin/env python3
"""
Test script to verify DirectionalLayer bit encoding
"""

def test_direction_bits():
    print("DirectionalLayer Bit Test:")
    print("=" * 40)
    
    directions = [
        ("East (→)", 0),
        ("Northeast (↗)", 1), 
        ("North (↑)", 2),
        ("Northwest (↖)", 3),
        ("West (←)", 4),
        ("Southwest (↙)", 5),
        ("South (↓)", 6),
        ("Southeast (↘)", 7)
    ]
    
    for name, bit in directions:
        mask_value = 1 << bit  # Create mask with only this bit set
        print(f"{name:15} | Bit {bit} | Mask: {mask_value:3d} (0b{mask_value:08b})")
    
    print("\nIf mask = 1 (0b00000001), only East should be allowed")
    print("If mask = 4 (0b00000100), only North should be allowed") 
    print("If mask = 5 (0b00000101), only East + North should be allowed")
    print("If mask = 255 (0b11111111), all directions should be allowed")

if __name__ == '__main__':
    test_direction_bits()
