#ifndef AXP20_H_
#define AXP20_H_

/*For AXP22*/
#define AXP_NAME                      "axp20"
#define AXP20_STATUS                  (0x00)
#define AXP20_MODE_CHGSTATUS          (0x01)
#define AXP20_OTG_STATUS              (0x02)
#define AXP20_IC_TYPE                 (0x03)
#define AXP20_DATA_BUFFER1            (0x04)
#define AXP20_DATA_BUFFER2            (0x05)
#define AXP20_DATA_BUFFER3            (0x06)
#define AXP20_DATA_BUFFER4            (0x07)
#define AXP20_DATA_BUFFER5            (0x08)
#define AXP20_DATA_BUFFER6            (0x09)
#define AXP20_DATA_BUFFER7            (0x0A)
#define AXP20_DATA_BUFFER8            (0x0B)
#define AXP20_DATA_BUFFER9            (0x0C)
#define AXP20_DATA_BUFFERA            (0x0D)
#define AXP20_DATA_BUFFERB            (0x0E)
#define AXP20_DATA_BUFFERC            (0x0F)
#define AXP20_LDO234_DC23_CTL         (0x12)
#define AXP20_DC2OUT_VOL              (0x23)
#define AXP20_LDO3_DC2_DVM            (0x25)
#define AXP20_DC3OUT_VOL              (0x27)
#define AXP20_LDO24OUT_VOL            (0x28)
#define AXP20_LDO3OUT_VOL             (0x29)
#define AXP20_IPS_SET                 (0x30)
#define AXP20_VOFF_SET                (0x31)
#define AXP20_OFF_CTL                 (0x32)
#define AXP20_CHARGE1                 (0x33)
#define AXP20_CHARGE2                 (0x34)
#define AXP20_BACKUP_CHG              (0x35)
#define AXP20_POK_SET                 (0x36)
#define AXP20_DCDC_FREQSET            (0x37)
#define AXP20_VLTF_CHGSET             (0x38)
#define AXP20_VHTF_CHGSET             (0x39)
#define AXP20_APS_WARNING1            (0x3A)
#define AXP20_APS_WARNING2            (0x3B)
#define AXP20_TLTF_DISCHGSET          (0x3C)
#define AXP20_THTF_DISCHGSET          (0x3D)
#define AXP20_DCDC_MODESET            (0x80)
#define AXP20_ADC_EN1                 (0x82)
#define AXP20_ADC_EN2                 (0x83)
#define AXP20_ADC_SPEED               (0x84)
#define AXP20_ADC_INPUTRANGE          (0x85)
#define AXP20_ADC_IRQ_RETFSET         (0x86)
#define AXP20_ADC_IRQ_FETFSET         (0x87)
#define AXP20_TIMER_CTL               (0x8A)
#define AXP20_VBUS_DET_SRP            (0x8B)
#define AXP20_HOTOVER_CTL             (0x8F)
#define AXP20_GPIO0_CTL               (0x90)
#define AXP20_GPIO0_VOL               (0x91)
#define AXP20_GPIO1_CTL               (0x92)
#define AXP20_GPIO2_CTL               (0x93)
#define AXP20_GPIO012_SIGNAL          (0x94)
#define AXP20_GPIO3_CTL               (0x95)
#define AXP20_INTEN1                  (0x40)
#define AXP20_INTEN2                  (0x41)
#define AXP20_INTEN3                  (0x42)
#define AXP20_INTEN4                  (0x43)
#define AXP20_INTEN5                  (0x44)
#define AXP20_INTSTS1                 (0x48)
#define AXP20_INTSTS2                 (0x49)
#define AXP20_INTSTS3                 (0x4A)
#define AXP20_INTSTS4                 (0x4B)
#define AXP20_INTSTS5                 (0x4C)

/* axp 20 adc data register */
#define AXP20_BAT_AVERVOL_H8          (0x78)
#define AXP20_BAT_AVERVOL_L4          (0x79)
#define AXP20_BAT_AVERCHGCUR_H8       (0x7A)
#define AXP20_BAT_AVERCHGCUR_L5       (0x7B)
#define AXP20_BAT_VOL_H8              (0x50)
#define AXP20_BAT_VOL_L4              (0x51)
#define AXP20_ACIN_VOL_H8             (0x56)
#define AXP20_ACIN_VOL_L4             (0x57)
#define AXP20_ACIN_CUR_H8             (0x58)
#define AXP20_ACIN_CUR_L4             (0x59)
#define AXP20_VBUS_VOL_H8             (0x5A)
#define AXP20_VBUS_VOL_L4             (0x5B)
#define AXP20_VBUS_CUR_H8             (0x5C)
#define AXP20_VBUS_CUR_L4             (0x5D)

#define AXP20_BAT_AVERDISCHGCUR_H8    (0x7C)
#define AXP20_BAT_AVERDISCHGCUR_L5    (0x7D)
#define AXP20_APS_AVERVOL_H8          (0x7E)
#define AXP20_APS_AVERVOL_L4          (0x7F)
#define AXP20_INT_BAT_CHGCUR_H8       (0xA0)
#define AXP20_INT_BAT_CHGCUR_L4       (0xA1)
#define AXP20_EXT_BAT_CHGCUR_H8       (0xA2)
#define AXP20_EXT_BAT_CHGCUR_L4       (0xA3)
#define AXP20_INT_BAT_DISCHGCUR_H8    (0xA4)
#define AXP20_INT_BAT_DISCHGCUR_L4    (0xA5)
#define AXP20_EXT_BAT_DISCHGCUR_H8    (0xA6)
#define AXP20_EXT_BAT_DISCHGCUR_L4    (0xA7)
#define AXP20_BAT_CHGCOULOMB3         (0xB0)
#define AXP20_BAT_CHGCOULOMB2         (0xB1)
#define AXP20_BAT_CHGCOULOMB1         (0xB2)
#define AXP20_BAT_CHGCOULOMB0         (0xB3)
#define AXP20_BAT_DISCHGCOULOMB3      (0xB4)
#define AXP20_BAT_DISCHGCOULOMB2      (0xB5)
#define AXP20_BAT_DISCHGCOULOMB1      (0xB6)
#define AXP20_BAT_DISCHGCOULOMB0      (0xB7)
#define AXP20_COULOMB_CTL             (0xB8)
#define AXP20_BAT_POWERH8             (0x70)
#define AXP20_BAT_POWERM8             (0x71)
#define AXP20_BAT_POWERL8             (0x72)

#define AXP20_VREF_TEM_CTRL           (0xF3)

/* bit definitions for AXP events ,irq event */
/*  AXP20  */
#define AXP20_IRQ_USBLO         (1)
#define AXP20_IRQ_USBRE         (2)
#define AXP20_IRQ_USBIN         (3)
#define AXP20_IRQ_USBOV         (4)
#define AXP20_IRQ_ACRE          (5)
#define AXP20_IRQ_ACIN          (6)
#define AXP20_IRQ_ACOV          (7)

#define AXP20_IRQ_TEMLO         (8)
#define AXP20_IRQ_TEMOV         (9)
#define AXP20_IRQ_CHAOV         (10)
#define AXP20_IRQ_CHAST         (11)
#define AXP20_IRQ_BATATOU       (12)
#define AXP20_IRQ_BATATIN       (13)
#define AXP20_IRQ_BATRE         (14)
#define AXP20_IRQ_BATIN         (15)

#define AXP20_IRQ_POKLO         (16)
#define AXP20_IRQ_POKSH         (17)
#define AXP20_IRQ_LDO3LO        (18)
#define AXP20_IRQ_DCDC3LO       (19)
#define AXP20_IRQ_DCDC2LO       (20)
#define AXP20_IRQ_CHACURLO      (22)
#define AXP20_IRQ_ICTEMOV       (23)

#define AXP20_IRQ_EXTLOWARN2    (24)
#define AXP20_IRQ_EXTLOWARN1    (25)
#define AXP20_IRQ_SESSION_END   (26)
#define AXP20_IRQ_SESS_AB_VALID (27)
#define AXP20_IRQ_VBUS_UN_VALID (28)
#define AXP20_IRQ_VBUS_VALID    (29)
#define AXP20_IRQ_PDOWN_BY_NOE  (30)
#define AXP20_IRQ_PUP_BY_NOE    (31)

#define AXP20_IRQ_GPIO0TG       (32)
#define AXP20_IRQ_GPIO1TG       (33)
#define AXP20_IRQ_GPIO2TG       (34)
#define AXP20_IRQ_GPIO3TG       (35)
#define AXP20_IRQ_PEKFE         (37)
#define AXP20_IRQ_PEKRE         (38)
#define AXP20_IRQ_TIMER         (39)

extern s32 axp_debug;
extern struct axp_config_info axp20_config;

#endif /* AXP20_H_ */
