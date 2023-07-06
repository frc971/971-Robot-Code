#include "aos/starter/irq_affinity_lib.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace aos::testing {

constexpr std::string_view kRockPiContents =
    R"contents(           CPU0       CPU1       CPU2       CPU3       CPU4       CPU5       
 23:    4221703    2325251    5737616    1332002    1418781    1368697     GICv3  30 Level     arch_timer
 25:    8034134    3985740    5155495    1708244    1835948    1635570     GICv3 113 Level     rk_timer
 31:      11137       2335       4349        978          0          0  GICv3-23   0 Level     arm-pmu
 32:          0          0          0          0        837        820  GICv3-23   1 Level     arm-pmu
 33:          0          0          0          0          0          0     GICv3  59 Level     rockchip_usb2phy
 34:          0          0          0          0          0          0     GICv3  63 Level     rockchip_usb2phy
 35:          0          0          0          0          0          0     GICv3  37 Level     ff6d0000.dma-controller
 36:          0          0          0          0          0          0     GICv3  38 Level     ff6d0000.dma-controller
 37:          0          0          0          0          0          0     GICv3  39 Level     ff6e0000.dma-controller
 38:   97326477          0          0          0          0          0     GICv3  40 Level     ff6e0000.dma-controller
 39:        240          0          0          0          0          0     GICv3 132 Level     ttyS2
 40:          0          0          0          0          0          0     GICv3 147 Level     ff650800.iommu
 41:          0          0          0          0          0          0     GICv3 149 Level     ff660480.iommu
 42:          0          0          0          0          0          0     GICv3 151 Level     ff8f3f00.iommu, ff8f0000.vop
 43:          0          0          0          0          0          0     GICv3 150 Level     ff903f00.iommu, ff900000.vop
 44:          0          0    3412401          0          0          0     GICv3  75 Level     ff914000.iommu, rkisp1
 45:          0          0          0          0          0          0     GICv3  76 Level     ff924000.iommu
 46:          0          0          0          0          0          0     GICv3  85 Level     ff1d0000.spi
 47:          0          0          0          0          0          0     GICv3  91 Level     ff110000.i2c
 48:          0          0          0          0          0          0     GICv3  66 Level     ff130000.i2c
 49:        264          0       1386          0          0          0     GICv3  89 Level     ff3c0000.i2c
 50:          0          0          0          0          0          0  rockchip_gpio_irq  21 Level     rk808
 56:          0          0          0          0          0          0     rk808   5 Edge      RTC alarm
 60:         54          0        357          0          0          0     GICv3  88 Level     ff3d0000.i2c
 61:          0          0          0          0          0          0     GICv3 152 Edge      ff848000.watchdog
 62:       9491      70670          0          0          0          0     GICv3  97 Level     dw-mci
 63:        207          0          0          0          0          0     GICv3  43 Level     mmc1
 64:          0          0          0          0          0          0  rockchip_gpio_irq   7 Edge      fe320000.mmc cd
 65:          0          0          0          0          0          0     GICv3 129 Level     rockchip_thermal
 66:          0          0          0          0          0          0     GICv3  94 Level     ff100000.saradc
 67:          0          0          0          0          0          0     GICv3  58 Level     ehci_hcd:usb1
 68:          0          0          0          0          0          0     GICv3  62 Level     ehci_hcd:usb2
 69:          0          0          0          0          0          0     GICv3  60 Level     ohci_hcd:usb3
 70:          0          0          0          0          0          0     GICv3  64 Level     ohci_hcd:usb4
 71:         37          0          0      33541          0          0     GICv3  44 Level     eth0
 72:          0          0          0          0          0          0     GICv3 137 Level     xhci-hcd:usb5
 73:      52061          0          0          0          0          0     GICv3 142 Level     xhci-hcd:usb7
 74:          0          0          0          0          0          0     GICv3  87 Level     ff680000.rga
 75:          0          0          0          0          0          0     GICv3 148 Level     ff660000.video-codec
 76:          0          0          0          0          0          0     GICv3 146 Level     ff650000.video-codec
 77:          0          0          0          0          0          0     GICv3 145 Level     ff650000.video-codec
 78:          0          0          0          0          0          0     GICv3  55 Level     ff940000.hdmi
 79:   50468710          0          0          0          0          0  rockchip_gpio_irq  16 Level     adis16505
IPI0:    629986   29367897    1604808    9830343    7634766    7528717       Rescheduling interrupts
IPI1:     11040      79148      22290      41912      29091      21995       Function call interrupts
IPI2:         0          0          0          0          0          0       CPU stop interrupts
IPI3:         0          0          0          0          0          0       CPU stop (for crash dump) interrupts
IPI4:   5391338    7612504    8510753   10479370   10766273   10712987       Timer broadcast interrupts
IPI5:   3040632     140996    4922363      50728      31487      24500       IRQ work interrupts
IPI6:         0          0          0          0          0          0       CPU wake-up interrupts
Err:          0
)contents";

constexpr std::string_view kRockPiContents2 =
    R"contents(           CPU0       CPU1       CPU2       CPU3       CPU4       CPU5       
 23:       1703      25251     737616       2002      18781     368697     GICv3  30 Level     arch_timer
 25:    8034134    3985740    5155495    1708244    1835948    1635570     GICv3 113 Level     rk_timer
 31:      11137       2335       4349        978          0          0  GICv3-23   0 Level     arm-pmu
 32:          0          0          0          0        837        820  GICv3-23   1 Level     arm-pmu
 33:          0          0          0          0          0          0     GICv3  59 Level     rockchip_usb2phy
 34:          0          0          0          0          0          0     GICv3  63 Level     rockchip_usb2phy
 35:          0          0          0          0          0          0     GICv3  37 Level     ff6d0000.dma-controller
 36:          0          0          0          0          0          0     GICv3  38 Level     ff6d0000.dma-controller
 37:          0          0          0          0          0          0     GICv3  39 Level     ff6e0000.dma-controller
 38:   97326477          0          0          0          0          0     GICv3  40 Level     ff6e0000.dma-controller
 39:        240          0          0          0          0          0     GICv3 132 Level     ttyS2
 40:          0          0          0          0          0          0     GICv3 147 Level     ff650800.iommu
 41:          0          0          0          0          0          0     GICv3 149 Level     ff660480.iommu
 42:          0          0          0          0          0          0     GICv3 151 Level     ff8f3f00.iommu, ff8f0000.vop
 43:          0          0          0          0          0          0     GICv3 150 Level     ff903f00.iommu, ff900000.vop
 44:          0          0    3412401          0          0          0     GICv3  75 Level     ff914000.iommu, rkisp1
 45:          0          0          0          0          0          0     GICv3  76 Level     ff924000.iommu
 46:          0          0          0          0          0          0     GICv3  85 Level     ff1d0000.spi
 47:          0          0          0          0          0          0     GICv3  91 Level     ff110000.i2c
 48:          0          0          0          0          0          0     GICv3  66 Level     ff130000.i2c
 49:        264          0       1386          0          0          0     GICv3  89 Level     ff3c0000.i2c
 50:          0          0          0          0          0          0  rockchip_gpio_irq  21 Level     rk808
 56:          0          0          0          0          0          0     rk808   5 Edge      RTC alarm
 60:         54          0        357          0          0          0     GICv3  88 Level     ff3d0000.i2c
 61:          0          0          0          0          0          0     GICv3 152 Edge      ff848000.watchdog
 62:       9491      70670          0          0          0          0     GICv3  97 Level     dw-mci
 63:        207          0          0          0          0          0     GICv3  43 Level     mmc1
 64:          0          0          0          0          0          0  rockchip_gpio_irq   7 Edge      fe320000.mmc cd
 65:          0          0          0          0          0          0     GICv3 129 Level     rockchip_thermal
 66:          0          0          0          0          0          0     GICv3  94 Level     ff100000.saradc
 67:          0          0          0          0          0          0     GICv3  58 Level     ehci_hcd:usb1
 68:          0          0          0          0          0          0     GICv3  62 Level     ehci_hcd:usb2
 69:          0          0          0          0          0          0     GICv3  60 Level     ohci_hcd:usb3
 70:          0          0          0          0          0          0     GICv3  64 Level     ohci_hcd:usb4
 71:         37          0          0      33541          0          0     GICv3  44 Level     eth0
 72:          0          0          0          0          0          0     GICv3 137 Level     xhci-hcd:usb5
 73:      52061          0          0          0          0          0     GICv3 142 Level     xhci-hcd:usb7
 74:          0          0          0          0          0          0     GICv3  87 Level     ff680000.rga
 75:          0          0          0          0          0          0     GICv3 148 Level     ff660000.video-codec
 76:          0          0          0          0          0          0     GICv3 146 Level     ff650000.video-codec
 77:          0          0          0          0          0          0     GICv3 145 Level     ff650000.video-codec
 78:          0          0          0          0          0          0     GICv3  55 Level     ff940000.hdmi
 79:   50468710          0          0          0          0          0  rockchip_gpio_irq  16 Level     adis16505
IPI0:    629986   29367897    1604808    9830343    7634766    7528717       Rescheduling interrupts
IPI1:     11040      79148      22290      41912      29091      21995       Function call interrupts
IPI2:         0          0          0          0          0          0       CPU stop interrupts
IPI3:         0          0          0          0          0          0       CPU stop (for crash dump) interrupts
IPI4:   5391338    7612504    8510753   10479370   10766273   10712987       Timer broadcast interrupts
IPI5:   3040632     140996    4922363      50728      31487      24500       IRQ work interrupts
IPI6:         0          0          0          0          0          0       CPU wake-up interrupts
Err:          0
)contents";

// Tests that the rock pi's /proc/interrupts is parseable.
TEST(InterruptsStatusTest, RockPi) {
  InterruptsStatus status;
  for (int i = 0; i < 2; ++i) {
    status.Update(kRockPiContents);
    EXPECT_EQ(status.states()[0].interrupt_number, 23);
    EXPECT_EQ(status.states()[0].chip_name, "GICv3");
    EXPECT_THAT(status.states()[0].count,
                ::testing::ElementsAre(4221703, 2325251, 5737616, 1332002,
                                       1418781, 1368697));
    EXPECT_EQ(status.states()[0].hwirq, "30");
    EXPECT_THAT(status.states()[0].actions,
                ::testing::ElementsAre("arch_timer"));

    EXPECT_EQ(status.states()[15].interrupt_number, 44);
    EXPECT_EQ(status.states()[15].chip_name, "GICv3");
    EXPECT_THAT(status.states()[15].count,
                ::testing::ElementsAre(0, 0, 3412401, 0, 0, 0));
    EXPECT_EQ(status.states()[15].hwirq, "75");
    EXPECT_THAT(status.states()[15].actions,
                ::testing::ElementsAre("ff914000.iommu", "rkisp1"));
  }

  status.Update(kRockPiContents2);

  EXPECT_EQ(status.states()[0].interrupt_number, 23);
  EXPECT_EQ(status.states()[0].chip_name, "GICv3");
  EXPECT_THAT(status.states()[0].count,
              ::testing::ElementsAre(1703, 25251, 737616, 2002, 18781, 368697));
  EXPECT_EQ(status.states()[0].hwirq, "30");
  EXPECT_THAT(status.states()[0].actions, ::testing::ElementsAre("arch_timer"));
}

constexpr std::string_view kAustinDesktopContents =
    R"contents(            CPU0       CPU1       CPU2       CPU3       CPU4       CPU5       CPU6       CPU7       CPU8       CPU9       CPU10      CPU11      CPU12      CPU13      CPU14      CPU15      CPU16      CPU17      CPU18      CPU19      
   8:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-IO-APIC    8-edge      rtc0
   9:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-IO-APIC    9-fasteoi   acpi
  14:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-IO-APIC   14-fasteoi   INTC1056:00
  16:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          4          0          0          0          0          0  IR-IO-APIC   16-fasteoi   peak_pciefd, peak_pciefd, peak_pciefd, peak_pciefd
  17:          0          0          0          0          0          0          0          0          0       1623          0          0          0          0          0          0          0          0          0          0  IR-IO-APIC   17-fasteoi   snd_hda_intel:card1
  18:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0        687          0  IR-IO-APIC   18-fasteoi   i801_smbus, snd_hda_intel:card2
  27:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-IO-APIC   27-fasteoi   idma64.0, i2c_designware.0
  29:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-IO-APIC   29-fasteoi   idma64.2, i2c_designware.2
  40:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-IO-APIC   40-fasteoi   idma64.1, i2c_designware.1
 120:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  DMAR-MSI    0-edge      dmar0
 121:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  DMAR-MSI    1-edge      dmar1
 136:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          1          0  IR-PCI-MSI 229376-edge      vmd0
 137:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229377-edge      vmd0
 138:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229378-edge      vmd0
 139:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229379-edge      vmd0
 140:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229380-edge      vmd0
 141:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229381-edge      vmd0
 142:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229382-edge      vmd0
 143:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229383-edge      vmd0
 144:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229384-edge      vmd0
 145:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229385-edge      vmd0
 146:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229386-edge      vmd0
 147:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229387-edge      vmd0
 148:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229388-edge      vmd0
 149:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229389-edge      vmd0
 150:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229390-edge      vmd0
 151:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229391-edge      vmd0
 152:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229392-edge      vmd0
 153:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229393-edge      vmd0
 154:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 229394-edge      vmd0
 155:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0        124  IR-PCI-MSI 5767168-edge      thunderbolt
 156:        124          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 5767169-edge      thunderbolt
 171:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3145728-edge      enp6s0
 172:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3145729-edge      enp6s0
 173:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3145730-edge      enp6s0
 174:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3145731-edge      enp6s0
 175:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3145732-edge      enp6s0
 176:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3145733-edge      enp6s0
 177:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3145734-edge      enp6s0
 178:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3145735-edge      enp6s0
 179:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3145736-edge      enp6s0
 181:          0   30132468          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 327680-edge      xhci_hcd
 182:          0          0          0          0          0     251984          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572864-edge      nvme0q0
 183:          0          0          0    1327707          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 376832-edge      ahci[0000:00:17.0]
 184:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 33030144-edge      xhci_hcd
 185:         42          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572865-edge      nvme0q1
 186:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572866-edge      nvme0q2
 187:          0          0          4          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572867-edge      nvme0q3
 188:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572868-edge      nvme0q4
 189:          0          0          0          0          9          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572869-edge      nvme0q5
 190:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572870-edge      nvme0q6
 191:          0          0          0          0          0          0         60          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572871-edge      nvme0q7
 192:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572872-edge      nvme0q8
 193:          0          0          0          0          0          0          0          0         43          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572873-edge      nvme0q9
 194:          0          0          0          0          0          0          0          0          0         34          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572874-edge      nvme0q10
 195:          0          0          0          0          0          0          0          0          0          0         13          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572875-edge      nvme0q11
 196:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572876-edge      nvme0q12
 197:          0          0          0          0          0          0          0          0          0          0          0          0         58          0          0          0          0          0          0          0  IR-PCI-MSI 1572877-edge      nvme0q13
 198:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572878-edge      nvme0q14
 199:          0          0          0          0          0          0          0          0          0          0          0          0          0          0         90          0          0          0          0          0  IR-PCI-MSI 1572879-edge      nvme0q15
 200:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0         10          0          0          0          0  IR-PCI-MSI 1572880-edge      nvme0q16
 201:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0         16          0          0          0  IR-PCI-MSI 1572881-edge      nvme0q17
 202:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0         35          0          0  IR-PCI-MSI 1572882-edge      nvme0q18
 203:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          1          0  IR-PCI-MSI 1572883-edge      nvme0q19
 204:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 1572884-edge      nvme0q20
 205:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          1          0          0  IR-PCI-MSI 4194304-edge      enp8s0
 206:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0      42818          0  IR-PCI-MSI 4194305-edge      enp8s0-TxRx-0
 207:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0      42467  IR-PCI-MSI 4194306-edge      enp8s0-TxRx-1
 208:          0          0          0          0          0          0          0          0          0          0          0          0          0      42734          0          0          0          0          0          0  IR-PCI-MSI 4194307-edge      enp8s0-TxRx-2
 209:          0          0          0          0          0          0          0          0          0          0          0          0          0          0      42817          0          0          0          0          0  IR-PCI-MSI 4194308-edge      enp8s0-TxRx-3
 210:          0          0          0          0          0          0          0          0        352          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 32768-edge      i915
 211:          0          0          0          0          0          0          0          0          0          0       2545          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 514048-edge      snd_hda_intel:card0
 212:          0          0          0          0          0          0          0          0          0          0          0         50          0          0          0          0          0          0          0          0  IR-PCI-MSI 360448-edge      mei_me
 213:          0          0          0          0          0          0          0          0          0          0          0          0     827795          0          0          0          0          0          0          0  IR-PCI-MSI 3670016-edge      iwlwifi:default_queue
 214:      44562          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670017-edge      iwlwifi:queue_1
 215:          0      37107          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670018-edge      iwlwifi:queue_2
 216:          0          0      33498          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670019-edge      iwlwifi:queue_3
 217:          0          0          0      29533          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670020-edge      iwlwifi:queue_4
 218:          0          0          0          0      21427          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670021-edge      iwlwifi:queue_5
 219:          0          0          0          0          0      26244          0          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670022-edge      iwlwifi:queue_6
 220:          0          0          0          0          0          0      22479          0          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670023-edge      iwlwifi:queue_7
 221:          0          0          0          0          0          0          0      84711          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670024-edge      iwlwifi:queue_8
 222:          0          0          0          0          0          0          0          0      22071          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670025-edge      iwlwifi:queue_9
 223:          0          0          0          0          0          0          0          0          0      47348          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670026-edge      iwlwifi:queue_10
 224:          0          0          0          0          0          0          0          0          0          0      30472          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670027-edge      iwlwifi:queue_11
 225:          0          0          0          0          0          0          0          0          0          0          0      31430          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670028-edge      iwlwifi:queue_12
 226:          0          0          0          0          0          0          0          0          0          0          0          0      23299          0          0          0          0          0          0          0  IR-PCI-MSI 3670029-edge      iwlwifi:queue_13
 227:          0          0          0          0          0          0          0          0          0          0          0          0          0     109672          0          0          0          0          0          0  IR-PCI-MSI 3670030-edge      iwlwifi:queue_14
 228:          0          0          0          0          0          0          0          5          0          0          0          0          0          0          0          0          0          0          0          0  IR-PCI-MSI 3670031-edge      iwlwifi:exception
 229:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0    4120250          0          0          0  IR-PCI-MSI 524288-edge      nvidia
 230:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0     138205          0          0          0          0  IR-PCI-MSI 1048576-edge      nvidia
 NMI:         92         28        100         10        103         11        107          6        243          6        258          6        157          6        141          6         83         42         24         14   Non-maskable interrupts
 LOC:    8019100    2732118    7530896    1145194    7715584    1787839    8532279     820360   10887596     601426   12029097     810678   11081674     781984   10455644     576294    5640682    3954630    2196579    1312346   Local timer interrupts
 SPU:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0   Spurious interrupts
 PMI:         92         28        100         10        103         11        107          6        243          6        258          6        157          6        141          6         83         42         24         14   Performance monitoring interrupts
 IWI:          1          0          0          0          0          0          0          0         24          0         45          0          0          0          0          0          0          0          0          0   IRQ work interrupts
 RTR:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0   APIC ICR read retries
 RES:     199167     103904     200210      70287     231170      96596     247332      58629     202889      49773     231943      50596     293734      60653     290320      64894     564256     359718     233415     190403   Rescheduling interrupts
 CAL:    1375045     393024     796832     151717     815698     182423     836372     114075    1150075     122234    1201956     117418     989111     117108     974193     104812     661099     523054     386579     295224   Function call interrupts
 TLB:     424698      66883     450735      56137     444850      46655     450670      39023     723259      44390     748901      39980     537343      36037     516461      34380     317421     255858     180013     130541   TLB shootdowns
 TRM:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0   Thermal event interrupts
 THR:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0   Threshold APIC interrupts
 DFR:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0   Deferred Error APIC interrupts
 MCE:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0   Machine check exceptions
 MCP:        270        271        271        271        271        271        271        271        271        271        271        271        271        271        271        271        271        271        271        271   Machine check polls
 ERR:          0
 MIS:          0
 PIN:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0   Posted-interrupt notification event
 NPI:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0   Nested posted-interrupt event
 PIW:          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0   Posted-interrupt wakeup event
)contents";

// Tests that Austin's desktop's /proc/interrupts is parsable.
TEST(InterruptsStatusTest, Desktop) {
  InterruptsStatus status;
  status.Update(kAustinDesktopContents);
  status.Update(kAustinDesktopContents);
  EXPECT_EQ(status.states()[0].interrupt_number, 8);
  EXPECT_EQ(status.states()[0].chip_name, "IR-IO-APIC");
  EXPECT_THAT(status.states()[0].count,
              ::testing::ElementsAre(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0));
  EXPECT_EQ(status.states()[0].hwirq, "8-edge");
  EXPECT_THAT(status.states()[0].actions, ::testing::ElementsAre("rtc0"));
}

}  // namespace aos::testing
