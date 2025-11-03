import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def load_and_compare_performance(sector_file, original_file):
    # Check if files exist
    if not os.path.exists(sector_file):
        print(f"Error: {sector_file} not found. Run cam_algo_sectors.py first.")
        return
    
    if not os.path.exists(original_file):
        print(f"Error: {original_file} not found.")
        return
    
    # Load data
    try:
        sector_data = pd.read_csv(sector_file)
        original_data = pd.read_csv(original_file)
        
        print("=== PERFORMANCE COMPARISON ===\n")
        
        # Basic stats
        sector_avg = sector_data['Processing_Time_ms'].mean()
        sector_fps = sector_data['FPS'].mean()
        original_avg = original_data['Processing_Time_ms'].mean()
        original_fps = original_data['FPS'].mean()
        
        print(f"SECTOR ALGORITHM:")
        print(f"  Average: {sector_avg:.2f}ms ({sector_fps:.1f} FPS)")
        print(f"  Min/Max: {sector_data['Processing_Time_ms'].min():.2f}ms / {sector_data['Processing_Time_ms'].max():.2f}ms")
        print(f"  Frames: {len(sector_data)}")
        
        print(f"\nORIGINAL ALGORITHM:")
        print(f"  Average: {original_avg:.2f}ms ({original_fps:.1f} FPS)")
        print(f"  Min/Max: {original_data['Processing_Time_ms'].min():.2f}ms / {original_data['Processing_Time_ms'].max():.2f}ms")
        print(f"  Frames: {len(original_data)}")
        
        # Performance improvement
        improvement = ((original_avg - sector_avg) / original_avg) * 100
        fps_improvement = ((sector_fps - original_fps) / original_fps) * 100
        
        print(f"\n=== IMPROVEMENT ===")
        print(f"Speed improvement: {improvement:.1f}% faster")
        print(f"FPS improvement: {fps_improvement:.1f}% higher")
        
        if improvement > 0:
            print(f"SECTOR ALGORITHM IS {improvement:.1f}% FASTER!")
        else:
            print(f"Sector algorithm is {abs(improvement):.1f}% slower")
        
        # Create comparison plots
        fig, (ax1, ax2, ax4) = plt.subplots(1, 3, figsize=(18, 6))
        
        # Plot 1: Processing time over frames
        ax1.plot(sector_data['Frame'], sector_data['Processing_Time_ms'], 'b-', alpha=0.7, label='Sector Algorithm')
        ax1.plot(original_data['Frame'], original_data['Processing_Time_ms'], 'r-', alpha=0.7, label='Original Algorithm')
        ax1.set_xlabel('Frame Number')
        ax1.set_ylabel('Processing Time (ms)')
        ax1.set_title('Processing Time Comparison')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: FPS comparison
        ax2.plot(sector_data['Frame'], sector_data['FPS'], 'b-', alpha=0.7, label='Sector Algorithm')
        ax2.plot(original_data['Frame'], original_data['FPS'], 'r-', alpha=0.7, label='Original Algorithm')
        ax2.set_xlabel('Frame Number')
        ax2.set_ylabel('FPS')
        ax2.set_title('FPS Comparison')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Box plot comparison
        data_to_plot = [sector_data['Processing_Time_ms'], original_data['Processing_Time_ms']]
        ax4.boxplot(data_to_plot, labels=['Sector', 'Original'])
        ax4.set_ylabel('Processing Time (ms)')
        ax4.set_title('Processing Time Box Plot')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('performance_comparison.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        print(f"\nComparison plots saved as: performance_comparison.png")
        
        # Statistical analysis
        print(f"\n=== STATISTICAL ANALYSIS ===")
        
        # Consistency (standard deviation)
        sector_std = sector_data['Processing_Time_ms'].std()
        original_std = original_data['Processing_Time_ms'].std()
        
        print(f"Consistency (lower std = more consistent):")
        print(f"  Sector: {sector_std:.2f}ms std")
        print(f"  Original: {original_std:.2f}ms std")
        
        if sector_std < original_std:
            print(f"Sector algorithm is {((original_std - sector_std)/original_std)*100:.1f}% more consistent")
        else:
            print(f"Sector algorithm is {((sector_std - original_std)/original_std)*100:.1f}% less consistent")
        
        # Performance percentiles
        print(f"\nPerformance Percentiles:")
        print(f"  Sector 95th percentile: {np.percentile(sector_data['Processing_Time_ms'], 95):.2f}ms")
        print(f"  Original 95th percentile: {np.percentile(original_data['Processing_Time_ms'], 95):.2f}ms")
        
    except Exception as e:
        print(f"Error loading data: {e}")

if __name__ == "__main__":
    sector_file_path = "sector_performance_data_no_visuals.csv"
    original_file_path = "performance_data_no_visuals.csv"
    load_and_compare_performance(sector_file_path, original_file_path)